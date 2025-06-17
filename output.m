% output.m
%   Create different outputs to compute trim and linear models
%
%  Revised:
%   1/30/2010 - RB 

function out = output(in)

 
    % relabel the inputs
    u       = in(4);
    v       = in(5);
    w       = in(6);
    phi     = in(7);
    theta   = in(8);
    psi     = in(9);
    p       = in(10);
    q       = in(11);
    r       = in(12);
 %   wind_x  = in(13);
 %   wind_y  = in(14);
 %   wind_z  = in(15);
    
    % compute the velocity relative to the air mass
    ur      = u;%-wind_x;
    vr      = v;%-wind_y;
    wr      = w;%-wind_z;
    
    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    Va    = sqrt(ur^2 + vr^2 + wr^2);
    alpha = atan2(wr,ur);
    beta  = atan2(vr,sqrt(ur^2+wr^2));
  
    out = [Va; alpha; beta; phi; theta; psi; p; q; r];
end



