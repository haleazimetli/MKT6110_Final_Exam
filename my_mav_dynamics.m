function varargout = my_mav_dynamics(varargin)

if (nargin==4 || nargin==5) && nargout<=1
    x     = varargin{1};
    delta = varargin{2};
    if nargin==4          
        wind = zeros(6,1);
        P    = varargin{3};
        dt   = varargin{4};
    else                 
        wind = varargin{3};
        P    = varargin{4};
        dt   = varargin{5};
    end
    x = real(x); delta = real(delta); wind = real(wind);  

    FM   = forces_moments(x,delta,wind,P.mass,P.gravity,P);
    F    = FM(1:3);   M = FM(4:6);

    xdot = sixdof_rhs(x,F,M,P);

    varargout{1} = x + dt*xdot;
    return
end

%% 
t     = varargin{1}; 
x     = varargin{2};
u     = varargin{3};            
flag  = varargin{4};
P     = varargin{5};

switch flag
    case 0          % sizes, init
        sizes = simsizes;
        sizes.NumContStates  = 12;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 12;
        sizes.NumInputs      = 10;   % 4 delta + 6 wind
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0  = x;                    
        str = [];
        ts  = [0 0];                 % continuous
        varargout = {sys,x0,str,ts};

    case 1         
        delta = u(1:4);
        wind  = u(5:10);
        FM    = forces_moments(x,delta,wind,P.mass,P.gravity,P);
        F     = FM(1:3);   M = FM(4:6);
        varargout{1} = sixdof_rhs(x,F,M,P);

    case 3        
        varargout{1} = x;            
    otherwise      
        varargout = {[]};
end
end
%%
function xdot = sixdof_rhs(x,F,M,P)
pn = x(1);  pe = x(2);  pd = -x(3); %
u  = x(4);  v  = x(5);  w  = x(6);
phi = x(7); the = x(8); psi = x(9);
p = x(10);  q  = x(11); r  = x(12);

Rbi = angle2dcm(psi,the,phi,'ZYX')';

pos_dot = Rbi' * [u; v; w];

uvw_dot = (1/P.mass)*F - cross([p;q;r],[u;v;w]);

Gamma = [ 1  sin(phi)*tan(the)  cos(phi)*tan(the);
          0  cos(phi)          -sin(phi);
          0  sin(phi)/cos(the)  cos(phi)/cos(the) ];
eul_dot = Gamma * [p; q; r];

J = [ P.Jx   0     -P.Jxz;
      0      P.Jy   0;
     -P.Jxz  0      P.Jz ];
omega_dot = J \ (M - cross([p;q;r], J*[p;q;r]));

xdot = [pos_dot ; uvw_dot ; eul_dot ; omega_dot];
end
