function meas = imu_sensor(x, P)

F = x(13:15);
Sg  = diag([1.01 0.99 1.02]);           
Mg  = deg2rad([ 0    0.8  -0.4;        
                0.6   0    0.3;
               -0.5   0.4  0  ]);
Sa  = diag([1.00 1.03 0.97]);
Ma  = deg2rad([ 0    1.0  -0.6;
                0.7   0    0.4;
               -0.4   0.2  0  ]);

sig_g = 0.005;               sig_a = 0.03*P.gravity;
sig_bg = 5e-5;               sig_ba = 5e-4*P.gravity;

persistent bg ba
if isempty(bg)
    bg = deg2rad([0.3;-0.25;0.1]);      % başlangıç bias
    ba = 0.05*P.gravity*[-.8;.4;.2];
end
bg = bg + sig_bg*sqrt(P.Ts)*randn(3,1);
ba = ba + sig_ba*sqrt(P.Ts)*randn(3,1);

omega  = x(10:12);                             % p,q,r
Rbi    = angle2dcm(x(9),x(8),x(7),'ZYX')';
a_b    = F/P.mass - Rbi*[0;0;P.gravity];  % body ivme

gyro = (eye(3)+Mg)*Sg*omega  + bg + sig_g*randn(3,1);
acc  = (eye(3)+Ma)*Sa*a_b    + ba + sig_a*randn(3,1);

meas = [ gyro;      % 3×1
        acc   ]; 
end


% function y = imu_sensor(x,P)
%   % x: 12×1 durum, F: 3×1 kuvvet
%   % P: parametre yapısı (gravity, Ts)
%   % --- bias + gürültü (sadeleştirilmiş) ---
%   persistent bg ba
%   if isempty(bg)
%     bg = zeros(3,1);    % gyro bias
%     ba = zeros(3,1);    % acc bias
%   end
%   % ideal ölçü
%   omega = x(10:12);  % p,q,r
%   F = x(13:15);
%   Rbi   = angle2dcm(x(9),x(8),x(7),'ZYX')';
%   a_b   = F/P.mass - Rbi*[0;0;P.gravity];
%   % ölçüm + basit gürültü
%   gyro = omega + 0.005*randn(3,1);
%   acc  = a_b + 0.03*P.gravity*randn(3,1);
%   y    = [gyro; acc];   % 6×1
% end

