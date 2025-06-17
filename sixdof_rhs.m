function xdot = sixdof_rhs(x, F, M, P)
% Sağ-taraf (body ekseni)
pn = x(1); pe = x(2); pd = x(3);
u  = x(4); v  = x(5); w  = x(6);
phi = x(7); the = x(8); psi = x(9);
p = x(10); q = x(11); r = x(12);

%  --- Kinematik ---
Rbi = angle2dcm(psi,the,phi,'ZYX')';
xdot(1:3,1) = Rbi'*[u;v;w];

%  --- Lineer hızlar ---
xdot(4:6,1) = (1/P.mass)*F - cross([p;q;r],[u;v;w]);

%  --- Euler açıları ---
Gamma = [1  sin(phi)*tan(the)  cos(phi)*tan(the);
         0  cos(phi)          -sin(phi);
         0  sin(phi)/cos(the)  cos(phi)/cos(the)];
xdot(7:9,1) = Gamma*[p;q;r];

%  --- Açısal hızlar ---
J = [ P.Jx  0     -P.Jxz;
      0    P.Jy    0   ;
     -P.Jxz 0    P.Jz ];
xdot(10:12,1) = J\(M - cross([p;q;r],J*[p;q;r]));
end