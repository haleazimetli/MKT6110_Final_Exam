function meas = gps_sensor(x, P)

persistent eta_n eta_e eta_h
if isempty(eta_n)
    eta_n = 0; eta_e = 0; eta_h = 0;
end
beta = exp(-P.Ts/1100);       

% Gauss-Markov 
eta_n = beta*eta_n + (1-beta)*randn;
eta_e = beta*eta_e + (1-beta)*randn;
eta_h = beta*eta_h + (1-beta)*randn;

pn = x(1) + eta_n*P.sig_gps_n;   
pe = x(2) + eta_e*P.sig_gps_e;
h  = -x(3) + eta_h*P.sig_gps_h;  

Vg = sqrt(x(4)^2+x(5)^2);       
X  = atan2(x(5),x(4));       

meas = [pn; pe; h; Vg + 0.05*randn; X + deg2rad(1)*randn];
%             ↑  σ≈0.05 m/s         ↑ σ≈1 °
end
% function y = gps_sensor(x,P)
%   pn = x(1) + 0.5*randn;
%   pe = x(2) + 0.5*randn;
%   h  = -x(3) + 0.3*randn;
%   Vg = norm(x(4:6)) + 0.05*randn;
%   chi= atan2(x(5),x(4)) + deg2rad(1)*randn;
%   y  = [pn; pe; h; Vg; chi];  % 5×1
% end