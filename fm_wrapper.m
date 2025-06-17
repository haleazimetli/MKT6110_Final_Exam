function [F, M] = fm_wrapper(x, delta, wind, P)
    FM = forces_moments(x, delta, wind, [], [], P);  % senin fonksiyon
    F = FM(1:3);
    M = FM(4:6);
end
