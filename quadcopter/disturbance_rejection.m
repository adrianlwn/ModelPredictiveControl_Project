function [ innerController, filter ] = disturbance_rejection(sys, N, x0, r, T, d)
%DISTURBANCE_REJECTION Summary of this function goes here
%   Detailed explanation goes here

%% Initializations
Qf = sys.LQRPenalty.weight;
Af = sys.LQRSet.A;
bf = sys.LQRSet.b;
Q = sys.x.penalty.H;
R = sys.u.penalty.H;
A = sys.A;
B = sys.B;
C = [eye(4), zeros(4,3)];

% sizes :
n_x = size(A,2);
n_u = size(B,2);
n_r = size(R,1);
n_d = n_x;

Bd = zeros(n_x,n_d);
Cd = zeros(n_r,n_d);

%% Design Estimator Using Pole Placement
A_hat = [A, eye(n_x); zeros(n_x), eye(n_x)];
B_hat = [B; zeros(n_x,1)];
C_hat = [eye(n_x),zeros(n_x)];

pz= 0.35; % altitude
pa= 0.3;  % fast convergence for roll which was slow
pb= 0.4;  % relatively fast convergence for yaw
pg= 0.8;  % not important to have fast convergence
%Anything in between
pad= 0.45;
pbd= 0.5;
pgd= 0.55;
pu1= 0.6;
pu2= 0.65;
pu3= 0.7;
pu4= 0.75;

poles_x = [pz;pa;pb;pg;pad;pbd;pgd]';
poles_u = [pu1,pu2,pu3,pu4];

F_kalman = [poles_x,poles_u]; %Study what the values do
L = -place(A_hat',C_hat',F_kalman)';

filter.Af = A_hat-L*C_hat;
filter.Bf = [B_hat, L];

%% Define optimization variables
dx = sdpvar(n_x,N,'full');
du = sdpvar(n_u,N,'full');
ref = sdpvar(n_r,1,'full');
xs = sdpvar(n_x,1,'full');
us = sdpvar(n_u,1,'full');
x_est = sdpvar(n_x,1,'full');
d_est = sdpvar(n_d,1,'full');
x_init = sdpvar(n_x,1,'full');
d_est_init = sdpvar(n_d,1,'full');
u_init = sdpvar(n_u,1,'full');

%% MPC Delta Tracking

% Define constraints and objective
con = [];
obj = us'*us; % us'*Rs*us; % Terminal weight

% Estimator Constraints
% "measure of y" xm : true state
con = [con , u_init == du(:,1) + us ]

% x and d estimation
con = [con, [x_est ; d_est] == filter.Af * [x_init ; d_est_init] + filter.Bf * [u_init ; x_init]]; ]


% Steady State Constraints
con = [con, ([eye(n_x) - A , -B ; C , zeros( size(C,1), size(B,2))]*[xs;us] == [Bd*d_est;ref - Cd*d_est])]; % System dynamics
con = [con, sys.u.min <= us <= sys.u.max ]; % Input constraints
con = [con, sys.x.min <= xs <= sys.x.max ]; % State constraints

con = [con , dx(:,1) ==  x_est - xs ];

for k = 1:N-1
    
    % MPC Delta Formulation Constraints
    con = [con, (dx(:,k+1) == sys.A*dx(:,k) + sys.B*du(:,k))]; % System dynamics
    con = [con, sys.x.min <= dx(:,k) + xs  <= sys.x.max ]; % State constraints
    con = [con, sys.u.min <= du(:,k) + us <= sys.u.max ]; % Input constraints
    obj = obj + dx(:,k)'*Q*dx(:,k) + du(:,k)'*R*du(:,k); % Cost function
end

con = [con, Af*dx(:,N) <= bf ]; % Terminal constraint
obj = obj + dx(:,N)'*Qf*dx(:,N); % Terminal weight

% Compile the matrices
ops = sdpsettings('verbose',1,'solver','quadprog');


innerController = optimizer(con, obj, ops, [x_init ; ref ; d_est_init], du(:,1) + us);
simQuad( sys, innerController, x0, T , r);


end

