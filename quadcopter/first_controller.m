function [ sys ] = first_controller( sys,T,x0 , N, us )
%%% Cost x and u :
close all

Q = diag([60 6000 2000 1 0 0 0]);
R = 0.2*diag([1 1 1 1]);

%%% Terminal set and Terminal Weight

sys = LTISystem('A', sys.A, 'B', sys.B, 'Ts', sys.Ts);
sys.x.max = [1; deg2rad(10) ; deg2rad(10) ; 50*pi ;deg2rad(15) ; deg2rad(15) ; deg2rad(60)];
sys.x.min = -sys.x.max;

sys.u.max = [1;1;1;1]-us; 
sys.u.min = [0;0;0;0]-us;

sys.x.penalty = QuadFunction(Q);
sys.u.penalty = QuadFunction(R);

Qf = sys.LQRPenalty.weight;
Af = sys.LQRSet.A;
bf = sys.LQRSet.b;

%%% MPC

% Define optimization variables
x = sdpvar(7,N,'full');
u = sdpvar(4,N,'full');

% Define constraints and objective
con = [];
obj = 0;

for i = 1:N-1
    con = [con, (x(:,i+1) == sys.A*x(:,i) + sys.B*u(:,i))]; % System dynamics
    con = [con, (x(:,i) <= sys.x.max),(x(:,i) >= sys.x.min) ]; % State constraints
    con = [con, (u(:,i) <= sys.u.max), (u(:,i) >= sys.u.min)]; % Input constraints
    obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i); % Cost function
end

con = [con, (Af*x(:,N) <= bf)]; % Terminal constraint
obj = obj + x(:,N)'*Qf*x(:,N); % Terminal weight


options = sdpsettings('solver','quadprog');
innerController = optimizer(con, obj, options, x(:,1), u(:,1));

simQuad( sys, innerController, x0, T);


end

