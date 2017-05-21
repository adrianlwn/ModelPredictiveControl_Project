% loads:
%    hovering equilibrium (xs,us)
%    continuous time matrices Ac,Bc of the linearization
%    matrices sys.A, sys.B of the inner-loop discretized with sampling period sys.Ts
%    outer controller optimizer instance
load('quadData.mat')
outerController = getOuterController(Ac, 'cplex');
disp('Data successfully loaded')

%% %%%%%%%%%%%%%% First MPC controller %%%%%%%%%%%%%%%%%%%
T = 10; % simulation time [s]
x0 = [ -1 deg2rad(10) deg2rad(-10) deg2rad(120) 0 0 0 ]';
N = ceil(2/sys.Ts);

sys = first_controller(sys,T,x0 , N,us );
%pause

%% Reference tracking - no disturbance, no invariant sets
fprintf('PART II - reference tracking...\n')
T = 10;
N = ceil(2/sys.Ts);

N_ref = T/sys.Ts;
n_ref = [sys.Ts:sys.Ts:T]';

constant_ref = [0 deg2rad(5) deg2rad(-5) deg2rad(60)]';
innerController = reference_tracking(sys, N, x0, constant_ref,T);

slow_ref = [sin(n_ref) deg2rad(5*sin(n_ref)) deg2rad(-5*sin(n_ref)) deg2rad(60*sin(n_ref))]';
innerController = reference_tracking(sys, N, x0, slow_ref,T);
<<<<<<< HEAD
pause

%% Nonlinear model simulation - no disturbance
fprintf('Running the FIRST NL model simulation...\n')

sim('simulation1.mdl') 

pause

%% Disturbance estimation
%estimator
% 
% 
% %% Offset free MPC
% fprintf('PART III - OFFSET FREE / Disturbance rejection...\n')
% 
% pause
% %% Final simulation
% fprintf('Running the FINAL NL model simulation...\n')
% %sim('simulation2.mdl') 
% pause
% %% BONUS - Slew rate constraints
% % run after doing nonlinear simulations otherwise the NL simulations won't
% % work (because of the additional controller argument)
% fprintf('BONUS - SLEW RATE CONSTRAINTS...\n')
% 
% 
% 
% 
% 
% 
=======

%pause

%% Nonlinear model simulation - no disturbance
fprintf('Running the FIRST NL model simulation...\n')

%sim('simulation1.mdl') 

%pause

%% Disturbance estimation
%estimator


%% Offset free MPC
fprintf('PART III - OFFSET FREE / Disturbance rejection...\n')

%pause
%% Final simulation
fprintf('Running the FINAL NL model simulation...\n')
%sim('simulation2.mdl') 
%pause
%% BONUS - Slew rate constraints
% run after doing nonlinear simulations otherwise the NL simulations won't
% work (because of the additional controller argument)
fprintf('BONUS - SLEW RATE CONSTRAINTS...\n')






>>>>>>> master
