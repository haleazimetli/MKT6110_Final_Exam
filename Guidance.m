function y = Guidance(u)
%% OUTPUTS
Va_c  = 0.0;
h_c   = 0.0;
chi_c_deg = 0.0;
chi_c = 0;
R_orbit  = 50;   
lambda    = 1;  
k_orbit   = 4;  
%% -- Defines
persistent WP WP_i mode
if isempty(mode)          
    WP   = [  0   0  30 13  0;
             200 200 100 15  1;
             400   0 100 15  0;
             600 200   0 13  0];
    WP_i = 1;
    mode = 0;
end

%% -- Input Variables
pn = u(1);  pe = u(2);
h  = -u(3);                     % NED → +h
u_b = u(4); v_b = u(5); w_b = u(6);
psi = u(9);
Va  = sqrt(u_b^2 + v_b^2 + w_b^2);

%% -- State Machine
R_switch = 15;    Va_nom = 13;

switch mode
  case 0   % TAKE-OFF
      Va_c = Va_nom;  h_c = 30;  chi_c = psi;
      if h > 25, mode = 1; end

  case 1   % CLIMB
      firstAlt = WP(1,3) - 20;
      Va_c = Va_nom;  h_c = firstAlt;  chi_c = psi;
      if h > firstAlt-2, mode = 2; end

case 2   % CRUISE or ORBIT
    if WP_i >= size(WP,1), mode = 3; WP_i = size(WP,1)-1; end
    
    w1 = WP(WP_i ,:);    w2 = WP(WP_i+1 ,:);
    
    if w2(5)==0   %–– düz çizgi
        chi_q = atan2(w2(2)-w1(2), w2(1)-w1(1));
        chi_c = wrapToPi(chi_q);
        Va_c  = w2(4);   h_c = w2(3);
        % WP switch
        dist2 = (pn-w2(1))^2 + (pe-w2(2))^2;
        if dist2 < R_switch^2, WP_i = WP_i+1; end
    
    else          %–– ORBIT
        cx = w2(1);     cy = w2(2);     
        d  = sqrt( (pn-cx)^2 + (pe-cy)^2 );
        
        chi_q = atan2(cy-pe, cx-pn) + lambda*pi/2;
        chi_c = wrapToPi(chi_q + lambda*atan(k_orbit*(d-R_orbit)/R_orbit));
        Va_c  = w2(4);   h_c = w2(3);
      
        if abs(d-R_orbit) < 2 && abs( wrapToPi(psi-chi_q) ) < deg2rad(5)
        
            WP_i = WP_i+1;  
        end
    end

  case 3   % LANDING
      Va_c = Va_nom*0.8;  chi_c = psi;  h_c = max(0, h-0.7);
end
      chi_c_deg = rad2deg(chi_c);

y = [Va_c; h_c; chi_c_deg];
end