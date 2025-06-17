function xhat = ekf_nav(u, P)
% EKF for MAV navigation
% u = [ imu_gyro(3); imu_acc(3); gps_pn; gps_pe; gps_h; gps_Vg; gps_chi ]
% P  = struct of parameters

  persistent x_prev P_prev
  if isempty(x_prev)
    % başlangıç durumu ve kovaryans
    x_prev = P.x_trim;  % 12×1
    P_prev = diag([1e2*ones(1,3), ...        % pos
                   1e1*ones(1,3), ...        % vel
                   (pi/180)^2*ones(1,3), ... % euler
                   1e-2*ones(1,3)]);        % omega
  end

  % sensör verilerini ayıkla
  imu_meas = u(1:6);      % [gyro; acc]
  gps_meas = u(7:11);     % [pn; pe; h; Vg; chi]

  %-------- 1) PREDICT --------------------------------------
  omega = imu_meas(1:3);  % p,q,r
  acc_b = imu_meas(4:6);  % body ivme

  % strapdown INS (1. mertebe)
  phi  = x_prev(7); th = x_prev(8); psi = x_prev(9);
  Rbi  = angle2dcm(psi,th,phi,'ZYX')';
  acc_i = Rbi'*acc_b + [0;0;-P.gravity];
  vel   = x_prev(4:6) + P.Ts*acc_i;
  pos   = x_prev(1:3) + P.Ts*vel;
  % açı entegrasyonu
  phi = phi + P.Ts*omega(1);
  th  = th  + P.Ts*omega(2);
  psi = psi + P.Ts*omega(3);

  x_pred = [pos; vel; phi; th; psi; omega];

  % lineerleştirme (Jacobian)
  F = ekf_F_jacobian(x_prev, omega, acc_b, P);  

  Q = 0.1*P.Q_imu;    % 12×12 proses gürültüsü
  P_pred = F*P_prev*F' + Q;

  %-------- 2) UPDATE (GPS) ----------------------------------
  % GPS her adım geliyorsa, güncelle:
  % (Eğer gerçekte bazı adımlarda GPS yoksa, o adımlarda gps_meas = NaN yapıp
  %  isnan kontrolü de koyabilirsin. Şimdilik hep update ediyoruz.)
  Vg   = norm(vel);
  chi  = atan2(vel(2),vel(1));
  z_pred = [ pos(1);
             pos(2);
            -pos(3);
             Vg;
             chi ];

  H = ekf_H_jacobian(x_pred, P);   % 5×12
  R = 10*P.R_gps;                     % 5×5

  y = gps_meas - z_pred;
  S = H*P_pred*H' + R;
  K = (P_pred*H')/S;

  x_upd = x_pred + K*y;
  P_upd = (eye(12) - K*H)*P_pred;

  % bir sonraki adım için sakla
  x_prev = x_upd;
  P_prev = P_upd;
  xhat   = x_upd;
end

function F = ekf_F_jacobian(x, omega, acc_b, P)
% Continuous‐time model:
%   x = [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
%   \dot x = f(x,u)
% Bunu Euler ile bir adım ilerletince: x_pred = x + Ts*f(x,imu)
% Dolayısıyla discrete Jacobian ≈ I + Ts*∂f/∂x
%
% Biz ∂f/∂x’yi basitleştirilmiş olarak alıyoruz:
phi   = x(7); th = x(8); ps = x(9);
p     = x(10); q  = x(11); r  = x(12);
Rbi   = angle2dcm(ps,th,phi,'ZYX')';

% ∂pos_dot/∂uvw = ∂(Rbi'*[u;v;w]) / ∂[u;v;w]
J_pos_vel = Rbi';

% ∂uvw_dot/∂[phi,theta,psi] sıfır kabul edilebilir (küçük açı varsayımı)
% ∂uvw_dot/∂uvw de hızlı değişmediği varsayılsın → sıfır

% ∂eul_dot/∂[phi,theta] var, ama küçük açı ≈ sıfır

% Sonuçta
F_cont = zeros(12);
% pn,pe,pd türevi
F_cont(1:3,4:6) = J_pos_vel;
% u,v,w türevi zaten cross-term’ler de aslında x’u × omega, onları ihmal edebiliriz
% euler açıları
F_cont(7:9,10:12) = eye(3);
% pqr türevi ihmal

% discrete Jacobian:
F = eye(12) + P.Ts * F_cont;
end

function H = ekf_H_jacobian(x_pred, P)
% GPS ölçüm modeli: z = [pn; pe; h; Vg; chi]
%   pn = x(1)
%   pe = x(2)
%   h  = -x(3)
%   Vg = sqrt(u^2 + v^2 + w^2)  (burada u=x(4),v=x(5),w=x(6))
%   chi = atan2(v,u)
%
H = zeros(5,12);

% pn, pe, h satırları:
H(1,1) = 1;
H(2,2) = 1;
H(3,3) = -1;

% Vg satırı:
u = x_pred(4);  v = x_pred(5);  w = x_pred(6);
Vg = sqrt(u^2+v^2+w^2) + eps;
H(4,4) = u/Vg;
H(4,5) = v/Vg;
H(4,6) = w/Vg;

% chi satırı:
% chi = atan2(v,u)  →  ∂chi/∂u = -v/(u^2+v^2),  ∂chi/∂v = u/(u^2+v^2)
den = u^2+v^2 + eps;
H(5,4) = -v/den;
H(5,5) =  u/den;
end