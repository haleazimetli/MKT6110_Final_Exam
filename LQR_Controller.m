
function y = LQR_Controller(u,P)
t = u(16);

persistent int_V      ; if isempty(int_V),      int_V = 0; end
persistent theta_filt ; if isempty(theta_filt), theta_filt = 0; end
persistent x_int      ; if t==0,                x_int = 0; end
persistent log        ; if t==0, log = []; end

if t==0
    x_int      = 0;
    log_data   = [];
    int_V      = 0;
    theta_filt = 0;
end
y = zeros(28,1);

%% ------------------------------------------------ 1. Referanslar
x      = u(1:12);
Va_c   = u(13);
h_c    = u(14);
chi_c  = deg2rad(u(15));
err_V  = Va_c - u(4);
h      = -x(3);
h_err  = h_c - h;                              

gamma_cmd  = atan(h_err/60);
theta_ref  = sat(0.35*gamma_cmd, 15*pi/180, -15*pi/180);
alpha  = 0.90;                                   
theta_filt = alpha*theta_filt + (1-alpha)*theta_ref;

%% Sapma vektörleri
x_lon   = [ x(4); x(6); x(11); x(8);  -x(3);  x_int ];
xref_lon= [ Va_c; 0; 0; theta_filt;   h_c;    0     ];
xi_lon  = x_lon - xref_lon;

x_lat   = [ x(5); x(10); x(12); x(7); x(9) ];
xref_lat= [ 0;    0;     0;     0;   chi_c ];
xi_lat  = x_lat - xref_lat;

%% Uzunlamasına kumandalar
u_lon   = -P.K_lon * xi_lon + P.u_trim_lon;    
Se_cmd  = u_lon(1);

% -------- Throttle--------------------- 

St_ff   = P.u_trim_lon(2);          % trim gaz (~0.35-0.4)
k_p_t   = 1.5;                      % önce 1.2
k_i_t   = 0.015;                     % önce 0.05
int_V  = sat(int_V + P.Ts * err_V , 25 , -25); 
St_PI   = k_p_t*err_V + k_i_t*int_V;
St_qd   = -0.01 * x(11);            
St_unsat = St_ff + St_PI + St_qd;   % tek formül
S_t      = sat(St_unsat , 1 , 0);   % asıl gaz

 
if abs(St_unsat-S_t) > 1e-3
    int_V = int_V - (St_unsat-S_t);
end


%% ------------------------------------------------ 4. Yanal kumandalar
u_lat   = -P.K_lat*xi_lat + P.u_trim_lat;
S_a     = sat(u_lat(1), P.delta_a_max, P.delta_a_min);
S_r     = sat(u_lat(2), P.delta_r_max, P.delta_r_min);

%% 5. Elevator & irtifa integrali
x_int = x_int + P.Ts * h_err;  
x_int = sat(x_int, 30, -30);
K_q       = 0.35;                             
S_e_unsat = Se_cmd - K_q*x(11);
S_e       = sat(S_e_unsat, P.delta_e_max, P.delta_e_min);
sat_margin = 0.9*P.delta_e_max;

if abs(S_e_unsat - S_e) > 1e-3
    x_int = x_int - (S_e_unsat - S_e);
end

%%
S = [S_e; S_a; S_r; S_t];
x_cmd = [0;0;-h_c; Va_c; 0;0; 0;0; chi_c; 0;0;0];

y(1:4)   = S;
y(5:16)  = x_cmd;
y(17:28) = x;

log = [log ; t, Va_c, u(4), St_PI, St_qd, St_unsat, S_t];
if t > 0
    assignin('base','log_debug',log);           
end
end

% ---------------- yardımcı fonksiyonlar -----------------------------
function z = sat(u,up,lo)
   z = min(max(u,lo),up);
end

