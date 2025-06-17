% once Workspace'e kaydedilen yapıyı alın
t = simout.time;         % zaman vektörü
z = simout.signals.values;   % N×5, sütun sırasıyla [pn pe h Vg chi]

% GPS pozisyon için standard sapma
sigma_pn = std( z(:,1) );
sigma_pe = std( z(:,2) );
sigma_h  = std( z(:,3) );

% GPS hız, heading
sigma_Vg  = std( z(:,4) );
sigma_chi = std( wrapToPi(z(:,5)) );  % açısal veriyi wrap ile düzelt

fprintf('GPS σ pn=%.2f m, pe=%.2f m, h=%.2f m, Vg=%.2f m/s, χ=%.2f°\n',...
        sigma_pn, sigma_pe, sigma_h, sigma_Vg, rad2deg(sigma_chi) );


u = simout1.signals.values;  % N×6
gyro = u(:,1:3);   acc = u(:,4:6);

sigma_gyro = std(gyro);
sigma_acc  = std(acc);

fprintf('IMU gyro σ = [%.4f, %.4f, %.4f] rad/s\n', sigma_gyro);
fprintf('IMU acc  σ = [%.3f, %.3f, %.3f] m/s²\n', sigma_acc);