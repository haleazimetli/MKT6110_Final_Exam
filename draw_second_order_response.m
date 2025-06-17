zeta = 0:.2:1;

figure(3), clf

wn = 1;
for i=1:length(zeta),
    sys = tf([wn^2],[1,2*zeta(i)*wn,wn^2]);
    [y,t]=step(sys,0:.01:20);
    subplot(211), plot(t,y), hold on
    [y,t] = impulse(sys,0:.01:20);
    subplot(212), plot(t,y), hold on
end

subplot(211)
    title('step response')
    ylabel('y(t)')
    xlabel('w_n t')
subplot(212)
    title('impulse response')
    ylabel('y(t)')
    xlabel('w_n t')