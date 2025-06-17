%% === test_longitude.m ============================================

clear all; close all; clc

run('param.m');               
Tf = 60;                      
dt = P.Ts;  
N  = round(Tf/dt);

%% ----------- Komut profilleri ------------------------------------
Va_step = 5;                   % +5 m/s step
h_step  = 20;                  % +20 m  step

Va_c = P.Va * ones(1,N);       % başlangıçta trim hız
h_c  = zeros(1,N);             % zemin irtifa

t_step = 5;                    % 5. saniyede basamak uygula
k0     = round(t_step/dt);

Va_c(k0:end) = P.Va + Va_step;
h_c(k0:end)  = h_step;
chi_c        = zeros(1,N);     

%% ----------- Sim döngüsü -----------------------------------------
x = P.x_trim;                  % 12-state başlangıç
h_log = nan(1,N); Va_log = nan(1,N); theta_log = nan(1,N);
log_Va =[];
log_delta_e     = [];                % [t  h_c   h]
log_theta = [];                % [t  theta_c  theta]
log_thr = [];
for k = 1:N
    t = (k-1)*dt;
    uu = [x ; Va_c(k) ; h_c(k) ; chi_c(k) ; t];   % 16×1
    %y  = LQR_Controller(uu,P);     
    y  = LQR_Controller(uu,P);     % PID Kontolcu
    delta = y(1:4);                

    wind = zeros(6,1);             % şimdilik 0 (steady+gust)

    x = my_mav_dynamics(x, delta, wind, P, dt);

    % log
    Va_log(k)    = sqrt(x(4)^2+x(5)^2+x(6)^2);
    h_log(k)     = -x(3);
    theta_log(k) = x(8)*180/pi;

     if mod(k,10)==1        % ~0.1 s örnekle
         theta_c = y(12)/P.K_theta_DC;        
    log_theta  (end+1,:) = [t, theta_c*180/pi, x(8)*180/pi];
    log_delta_e(end+1,:) = [t, delta(1)*180/pi];
    %log_state  (end+1,:) = [t, altitude_state];
    log_thr    (end+1,:) = [t, delta(4), Va_log(k)];
    %log_Va(end+1,:) = [t, Va_c(k), Va, delta_t];
end
end


%% ----------- Grafikler -------------------------------------------
time = (0:N-1)*dt;

figure;
subplot(3,1,1); hold on; grid on
plot(time, Va_log, 'b', time, Va_c, 'r--');
ylabel('V_a [m/s]'); title('Longitudinal step test');
legend('gerçek','komut','Location','SouthEast');

subplot(3,1,2); hold on; grid on
plot(time, h_log, 'b', time, h_c, 'r--');
ylabel('h [m]');

subplot(3,1,3); grid on
plot(time, theta_log, 'b');
ylabel('\theta [deg]'); xlabel('time [s]');
% ===============================================================
