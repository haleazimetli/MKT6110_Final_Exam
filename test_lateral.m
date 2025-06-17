% === test_lateral.m ================================================
clear all; close all; clc
run('param.m') ;                % P ve kazançlar
Tf = 30;                       % toplam süre
dt = P.Ts;  N = round(Tf/dt);

% Komutlar
Va_c  = P.Va;                  % sabit hız komutu
h_c   = 0;                     % sabit irtifa
chi_c = zeros(1,N);            % 5. saniyede 20° step
chi_c(round(5/dt):end) = 20;

% Başlangıç durumu ve log
x = P.x_trim;  phi=nan(1,N); psi=nan(1,N);

wind_steady  = [  0 ;   6 ; 0 ];   
wind_gust    = [  0 ;   0 ; 0 ];   
wind         = [wind_steady ; wind_gust];   % 6×1

for k=1:N
    t = (k-1)*dt;
    uu = [x; Va_c; h_c; chi_c(k); t];
    y  = LQR_Controller(uu,P);
    delta = y(1:4);                % kontrol yüzeyleri

    % --------- BURAYI KENDİ MODELİNLE DEĞİŞTİR -------------
    x = my_mav_dynamics(x, delta, wind, P, dt);   % <- kendi fonksiyonun
    % --------------------------------------------------------

    phi(k) = x(7);    % roll açısı
    psi(k) = x(9);    % heading
end

% Grafik
time = (0:N-1)*dt;
figure;
subplot(2,1,1); plot(time, phi*180/pi); grid on
ylabel('\phi [deg]'); title('Roll response')
subplot(2,1,2); plot(time, psi*180/pi); grid on
ylabel('\psi [deg]'); xlabel('time [s]')
% =========================================================