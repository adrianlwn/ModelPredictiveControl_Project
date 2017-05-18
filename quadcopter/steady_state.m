function [ xs_f , us_f ] = steady_state(r,A,B,C,J,j,Rs,Bd,Cd,d_est )
%STEADY_STATE Summary of this function goes here
%   Detailed explanation goes here
n_x = size(A,2);
n_u = size(B,2);
n_r = size(R);

if nargin == 7
    dim_d = 1;
    Bd = zeros(n_x,dim_d);
    Cd = zeros(n_r,dim_d);
    d_est = zeros(1,dim_d);
end

% Define optimization variables
xs = sdpvar(n_x,1,'full');
us = sdpvar(n_u,1,'full');
% Define constraints and objective
con = [];
con = [con, ([eye(n_x) - A , -B ; C , zeros(size(B,1), size(C,2))]*[xs;us] == [Bd*d_est;r - Cd*d_est])]; % System dynamics
con = [con, (J*us <= j)]; % Input constraints
obj =  us'*Rs*us; % Terminal weight

% Compile the matrices
ops = sdpsettings('solver','quadprog');
%ops.quadprog.TolCon = 10^-15;
ctrl = optimizer(con, obj,ops, [] , [xs ; us]);

[xu,isfeasible] = ctrl{[]};
xs_f = xu(1:n_x) ;
us_f = xu(n_x+1:end);

end

