function y = autopilot(uu,P)
%
% autopilot for mavsimhead
% 
% Modification History:
%   2/11/2010 - RWB
% 

% u_trim = [P.u_trim_lon(1);   % δe0
%           P.u_trim_lat(1);   % δa0
%           P.u_trim_lat(2);   % δr0
%           P.u_trim_lon(2)];  % δt0
% y = [ u_trim;            % 4 kumanda
%       zeros(12,1);       % x_command (dummy)
%       zeros(12,1) ];               % xhat = gerçek x (ya da zeros(12,1))
    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = -uu(3+NN); % inertial Down position
    u        = uu(4+NN);  % inertial velocity along body x-axis
    v        = uu(5+NN);  % inertial velocity along body y-axis
    w        = uu(6+NN);  % inertial velocity along body z-axis
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    psi      = uu(9+NN);  % yaw angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    NN = NN+12;
    Va_c     = uu(1+NN);  % commanded airspeed
    h_c      = uu(2+NN);  % commanded altitude
    chi_c    = pi/180*uu(3+NN);  % commanded heading 
    NN = NN+3;
    t        = uu(1+NN);   % time

    % estimate airspeed (note that wind is not accounted for)
    Va       = sqrt(u^2 + v^2 + w^2);


    %----------------------------------------------------------
    % lateral autopilot


    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = coordinated_turn_hold(v, 1, P);
        % use commanded roll angle to regulate heading
        phi_c   = heading_hold(chi_c, psi, r, 1, P);
        % use aileron to regulate roll angle
        delta_a = roll_hold(phi_c, phi, p, 1, P);     

    else
        delta_r = coordinated_turn_hold(v, 0, P);
        phi_c   = heading_hold(chi_c, psi, r, 0, P);
        %phi_c = 15*pi/180;    
        delta_a = roll_hold(phi_c, phi, p, 0, P);
    end


    %----------------------------------------------------------
    % longitudinal autopilot

    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end

    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = 1;
            theta_c = 30*pi/180;
            if h>=P.altitude_take_off_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end

        case 2,  % climb zone
            delta_t = 1;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h>=h_c-P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            elseif h<=P.altitude_take_off_zone,
                altitude_state = 1;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end

        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h<=h_c+P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
        case 4, % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            if h<=h_c-P.altitude_hold_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            elseif h>=h_c+P.altitude_hold_zone,
                altitude_state = 3;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
    end

    if t==0,
        delta_e = pitch_hold(theta_c, theta, q, 1, P);
    else
        delta_e = pitch_hold(theta_c, theta, q, 0, P);
    end




    %----------------------------------------------------------
    % create outputs

    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...           % pn
        0;...           % pe
        -h_c;...        % pd
        Va_c;...        % u
        0;...           % v
        0;...           % w
        phi_c;...       % phi
        theta_c*P.K_theta_DC;...           % theta
        chi_c;...       % psi
        0;...           % p
        0;...           % q
        0;...           % r
        ];
    % estimated states
    xhat = [...
        pn;...          % pn
        pe;...          % pe
        -h;...          % pd
        u;...           % u
        v;...           % v
        w;...           % w
        phi;...         % phi
        theta;...       % theta
        psi;...         % psi
        p;...           % p
        q;...           % q
        r;...           % r
        ];

    y = [delta; x_command; xhat];

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% heading_hold
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = heading_hold(chi_c, psi, r, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end

  % compute the current error
  error = chi_c - psi;

  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

  % proportional term
  up = P.heading_kp * error;

  % integral term
  ui = P.heading_ki * integrator;

  % derivative term
  ud = -P.heading_kd*r;


  % implement PID control
  phi_c = sat(up + ui + ud, 45*pi/180, -45*pi/180);

  % update persistent variables
  error_d1 = error;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end

  % compute the current error
  error = phi_c - phi;

  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

  % proportional term
  up = P.roll_kp * error;

  % integral term
  ui = P.roll_ki * integrator;

  % derivative term
  ud = -P.roll_kd*p;


  % implement PID control
  delta_a = sat(up + ui + ud, P.delta_a_max, P.delta_a_min);

  if P.roll_ki ~= 0
    integrator = integrator + (P.Ts/P.roll_ki)*(delta_a - (up + ui + ud));
  end
  % update persistent variables
  error_d1 = error;
 % fprintf('phi_c = %.2f deg, phi = %.2f deg, delta_a = %.2f deg\n', ...
          %rad2deg(phi_c), rad2deg(phi), rad2deg(delta_a));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, flag, P)
%% Implement the ptch_hold controller
  persistent integrator error_d1
  if flag==1        % simülasyon başında sıfırla
      integrator = 0;
      error_d1   = 0;
  end

  % ---- PID Calculate ------------------------------------------
  error = theta_c - theta;                        % θ-hata
  % integrator – trapezoidal rule
  integrator = integrator + (P.Ts/2)*(error + error_d1);

  up = P.pitch_kp * error;
  ui = P.pitch_ki * integrator;
  ud = -P.pitch_kd * q;

  delta_e_unsat = up + ui + ud;

  delta_e = sat(delta_e_unsat, P.delta_e_max, P.delta_e_min);

  % ---- anti-windup (back-calculation) ----------------------
  if P.pitch_ki ~= 0
      integrator = integrator + (P.Ts/P.pitch_ki)*(delta_e - delta_e_unsat);
  end
  error_d1 = error; % just in case you run the sim without the code 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end

  % compute the current error
  error = Va_c - Va;

  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);

  % proportional term
  up = P.airspeed_pitch_kp * error;

  % integral term
  ui = P.airspeed_pitch_ki * integrator;

  % derivative term
  ud = P.airspeed_pitch_kd * differentiator;


  % implement PID control
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);

  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end

  % compute the current error
  error = Va_c - Va;

  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);

  % proportional term
  up = P.airspeed_throttle_kp * error;

  % integral term
  ui = P.airspeed_throttle_ki * integrator;

  % derivative term
  ud = P.airspeed_throttle_kd * differentiator;


  % implement PID control
  delta_t = sat(P.u_trim(4)+up + ui + ud, 1, 0);

  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end

  % compute the current error
  error = h_c - h;

  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);

  % proportional term
  up = P.altitude_kp * error;

  % integral term
  ui = P.altitude_ki * integrator;

  % derivative term
  ud = P.altitude_kd * differentiator;

  theta_unsat = up + ui + ud;
  % implement PID control
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);

   if P.altitude_ki ~= 0
    integrator = integrator + (P.Ts/P.altitude_ki)*(theta_c - theta_unsat);
  end
  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_r = coordinated_turn_hold(v, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end

  % compute the current error
  error = -v;

  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule

  % proportional term
  up = P.sideslip_kp * error;

  % integral term
  ui = P.sideslip_ki * integrator;

  % derivative term
  ud = 0;%-P.sideslip_kd * r;


  % implement PID control
  theta_r = sat(up + ui + ud, 30*pi/180, -30*pi/180);

  % update persistent variables
  error_d1 = error;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end