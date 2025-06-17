% compute natural frequency and damping coefficients for different aircraft
% modes


% short period mode
eig_lon = eig(A_lon);
short = eig_lon(2);
wn_short = abs(short);
zeta_short = -real(short)/wn_short;

% phugoid mode
phugoid = eig_lon(4);
wn_phugoid = abs(phugoid);
zeta_phugoid = -real(phugoid)/wn_phugoid;

% dutch-roll mode
eig_lat = eig(A_lat);
dutchroll = eig_lat(4);
wn_dutchroll = abs(dutchroll);
zeta_dutchroll = -real(dutchroll)/wn_dutchroll;

% roll mode
rollmode = eig_lat(3);

% spiral mode
spiral = eig_lat(2);


