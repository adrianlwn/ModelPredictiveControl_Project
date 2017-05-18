function [ innerController ] = reference_tracking( sys )
Qf = sys.LQRPenalty.weight;
Af = sys.LQRSet.A;
bf = sys.LQRSet.b;
Q = sys.x.penalty.H;
R = sys.u.penalty.H;

% sizes :
n_x = size(A,2);
n_u = size(B,2);
n_r = size(R);
n_d = n_x;

Bd = zeros(n_x,n_d);
Cd = zeros(n_r,n_d);
d_est = zeros(n_d,1);

% Define optimization variables
dx = sdpvar(2,N,'full');
du = sdpvar(1,N,'full');
ref = sdvvar(4,1,'full');
xs = sdpvar(n_x,1,'full');
us = sdpvar(n_u,1,'full');

% Define constraints for the
con = [];
con = [con, ([eye(n_x) - A , -B ; C , zeros(size(B,1), size(C,2))]*[xs;us] == [Bd*d_est;ref - Cd*d_est])]; % System dynamics
con = [con, sys.u.min <= us <= sys.u.max ]; % Input constraints
con = [con, sys.x.min <= xs <= sys.x.max ]; % State constraints
obj = us'*us; % us'*Rs*us; % Terminal weight

% Define constraints and objective
con = [con , dx(:,1) ==  x_init - xs ];

for k = 1:N-1
    con = [con, (dx(:,k+1) == sys.A*dx(:,k) + sys.B*du(:,k))]; % System dynamics
    con = [con, sys.x.min <= dx(:,k) + xs  <= sys.x.max ]; % State constraints
    con = [con, sys.u.min <= du(:,k) + us <= sys.u.max ]; % Input constraints
    obj = obj + dx(:,k)'*Q*dx(:,k) + du(:,k)'*R*du(:,k); % Cost function
    
end
con = [con, Af*dx(:,N) <= bf ]; % Terminal constraint
obj = obj + dx(:,N)'*Qf*dx(:,N); % Terminal weight

% Compile the matrices
ops = sdpsettings('verbose',0,'solver','cplex');


innerController = optimizer(cons, obj, options, [x_init; ref ], du(:,1) + us);
simQuad( sys, innerController, x0, T);

end

