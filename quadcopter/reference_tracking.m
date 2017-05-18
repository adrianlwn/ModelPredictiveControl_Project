function [ innercontroler ] = reference_tracking( sys )
Qf = sys.LQRPenalty.weight;
Af = sys.LQRSet.A;
bf = sys.LQRSet.b;

% "measure of y" xm : true state
xm = A*xm(:,i-1)+B*u(i-1);
y = C*xm(:,i-1)+Cd*d0;

% x and d estimation
x_est = A*x_est(:,i-1)+B*u(i-1)+L(1:2)*(C*x_est(:,i-1) + Cd*d_est(i-1) - y);
d_est = d_est(i-1)+L(3)*(C*x_est(:,i-1) + Cd*d_est(i-1) - y);

% compute steady state
[xs_f , us_f] = steady_state(r,A,B,C,Bd,Cd,d_est,J,j,Rs);

% define opt variable
dx = sdpvar(2,N,'full');
du = sdpvar(1,N,'full');

dx0 = x_est - xs_f;
% Define constraints and objective
con = [dx(:,1) == dx0 ];
obj = 0;
for k = 1:N-1
    con = [con, (dx(:,k+1) == A*dx(:,k) + B*du(:,k))]; % System dynamics
    %con = [con, (F*dx <= f)]; % State constraints
    con = [con, (J*du(:,k) <= j - J*us_f)]; % Input constraints
    obj = obj + dx(:,k)'*Q*dx(:,k) + du(:,k)'*R*du(:,k); % Cost function
end
con = [con, (Af*dx(:,N) <= bf)]; % Terminal constraint
obj = obj + dx(:,N)'*Qf*dx(:,N); % Terminal weight

% Compile the matrices
ops = sdpsettings('verbose',0,'solver','quadprog');
diag = optimize(con, obj,ops);
if ~(diag.problem == 0)
    disp('We have a problem')
end
u = us_f + value(du(1));

innercontroler = optimizer(cons, obj, options, x(:,1), u(:,1));

end

