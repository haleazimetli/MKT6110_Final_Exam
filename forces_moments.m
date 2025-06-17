% forces_moments.m
%   Computes the foreces and moments acting on the airframe. 
%
%  Revised:
%   2/2/2010 - RB 

function f_m_out = forces_moments(x, delta, wind, mass, gravity, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % convert steady inertial frame wind to the body frame
    R = [...
        1, 0, 0;...
        0, cos(phi), sin(phi);...
        0, -sin(phi), cos(phi)]*...
        [...
        cos(theta), 0, -sin(theta);...
        0, 1, 0;...
        sin(theta), 0, cos(theta)]*...
        [...
        cos(psi), sin(psi), 0;...
        -sin(psi), cos(psi), 0;...
        0, 0, 1];
    u_w = R(1,:)*[u_wg; v_wg; w_wg] + u_wg;
    v_w = R(2,:)*[u_wg; v_wg; w_wg] + v_wg;
    w_w = R(3,:)*[u_wg; v_wg; w_wg] + w_wg;
    
    % compute the velocity relative to the air mass
    ur      = u-u_w;
    vr      = v-v_w;
    wr      = w-w_w;
    
    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    Va    = sqrt(ur^2 + vr^2 + wr^2);
    alpha = atan2(wr,ur);
    beta  = atan2(vr,sqrt(ur^2+wr^2));
    qbar = 0.5*P.rho*Va^2;
    ca    = cos(alpha);
    sa    = sin(alpha);
   
    % compute gravitaional forces
    Force(1) = -P.mass*P.gravity*sin(theta);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi);
    
    % compute Lift and Drag forces
    tmp1 = exp(-P.M*(alpha-P.alpha0));
    tmp2 = exp(P.M*(alpha+P.alpha0));
    sigma = (1+tmp1+tmp2)/((1+tmp1)*(1+tmp2));
    CL = (1-sigma)*(P.C_L_0+P.C_L_alpha*alpha);
    CD = P.C_D_0 + (1-sigma)*P.epsilon*(P.C_L_0+P.C_L_alpha*alpha)^2;
    if alpha>=0, 
        CL = CL + sigma*2*sa*sa*ca;
        CD = CD + sigma*2*sa*sa*sa;
    else
        CL = CL - sigma*2*sa*sa*ca;
        CD = CD - sigma*2*sa*sa*sa;
    end
    
    % compute aerodynamic forces
    Force(1) = Force(1) + qbar*P.S_wing*(-CD*ca + CL*sa);
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_q*ca + P.C_L_q*sa)*P.c*q/(2*Va);
    
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta);
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_p*p + P.C_Y_r*r)*P.b/(2*Va);
    
    Force(3) = Force(3) + qbar*P.S_wing*(-CD*sa - CL*ca);
    Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_q*sa - P.C_L_q*ca)*P.c*q/(2*Va);
     
    % compute aerodynamic torques
    
    Torque(1) = qbar*P.S_wing*P.b*(P.C_ell_0 + P.C_ell_beta*beta);
    Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_p*p + P.C_ell_r*r)*P.b/(2*Va);

    Torque(2) = qbar*P.S_wing*P.c*(P.C_M_0 + P.C_M_alpha*alpha);
    Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_M_q*P.c*q/(2*Va);

    
    Torque(3) = qbar*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta);
    Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_p*p + P.C_n_r*r)*P.b/(2*Va);

    % compute control forces
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_delta_e*ca+P.C_L_delta_e*sa)*delta_e;
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_delta_e*sa-P.C_L_delta_e*ca)*delta_e;
     
    % compute control torques
    Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
    Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_M_delta_e*delta_e;
    Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
  
    % compute propulsion forces
    motor_temp = P.k_motor^2*delta_t^2-Va^2;
    Force(1) = Force(1) + 0.5*P.rho*P.S_prop*P.C_prop*motor_temp;
    
    f_m_out = [Force'; Torque'];

end



