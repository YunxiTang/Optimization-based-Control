%% This is a trajectory optimization for hybrid system
clc;clear;close all;
robot = hopper();
problem = struct;
problem.Nx = robot.Nx;
problem.Nu = robot.Nu;
problem.h = robot.h;
problem.Tf = 5.0;           % final time
problem.Nm = 5;             % Number of steps in each mode
problem.Nt = problem.Tf / problem.h + 1;    % Number of time steps
problem.Nmodes = (problem.Nt/problem.Nm); % Number of modes
t_hist = 0:problem.h:problem.Tf;
% Dimension of decision variable
problem.Nz = problem.Nx * problem.Nt + problem.Nu * (problem.Nt - 1);

%% constraints index
c_init_inds = 1:problem.Nx;
c_term_inds = (c_init_inds(end)+1):(c_init_inds(end)+problem.Nx);
c_dyn_inds = (c_term_inds(end)+1):(c_term_inds(end)+problem.Nx*(problem.Nt-1));
c_stance_inds = (c_dyn_inds(end)+1):(c_dyn_inds(end)+ceil(problem.Nmodes/2)*problem.Nm);
c_length_inds = (c_stance_inds(end)+1):(c_stance_inds(end)+problem.Nt * 2);
c_flight_inds = (c_length_inds(end)+1):(c_length_inds(end)++ceil(problem.Nmodes/2)*(problem.Nm));
m_nlp = c_flight_inds(end);
problem.c_init_inds = c_init_inds;
problem.c_term_inds = c_term_inds;
problem.c_dyn_inds = c_dyn_inds;
problem.c_stance_inds = c_stance_inds;
problem.c_length_inds = c_length_inds;
problem.c_flight_inds = c_flight_inds;
problem.m_nlp = m_nlp;
%% cost matrices
problem.Q = eye(robot.Nx);
problem.R = 0.001 * eye(robot.Nu);
problem.Q_f = 1 * problem.Q;
%% Reference trajectory
uref = kron(ones(1, problem.Nt-1), [robot.m1*robot.g; 0.0]);
xref = zeros(problem.Nx, problem.Nt);
xref(1,:) = linspace(-1.0,1.0,problem.Nt);
xref(2,:) = 1.0 + 0.7*sin(2*pi/2.0*t_hist);
xref(3,:) = linspace(-1.0,1.0,problem.Nt);
xref(5,2:end-5) = (2.0/problem.Tf)*ones(1, problem.Nt-6);
xref(7,2:end-5) = (2.0/problem.Tf)*ones(1, problem.Nt-6);
problem.xref = xref;
problem.uref = uref;

%% naive initialization
xguess = xref + 0.00*randn(problem.Nx,problem.Nt);
uguess = uref + 0.00*randn(problem.Nu,problem.Nt-1);
z_init = [reshape([xguess(:,1:(problem.Nt-1)); uguess],(problem.Nx+problem.Nu)*(problem.Nt-1),1); xguess(:,end)];
% Z_data = load('zInit.mat');
% z_init = Z_data.zSoln;
%% nonlinear constraints
[c, ceq] = collect_constraint(z_init, problem, robot);

%% 
opt_prob.objective = @(z) compute_traj_cost(z,problem);
opt_prob.x0 = z_init;
opt_prob.Aineq = [];
opt_prob.bineq = [];
opt_prob.Aeq = [];
opt_prob.beq = [];
opt_prob.lb = [];
opt_prob.ub =  [];
opt_prob.solver = 'fmincon';
opt_prob.options = optimoptions('fmincon','Display','iter',...
                                'OptimalityTolerance', 1e-6,'MaxFunctionEvaluations',1000000,...
                                'MaxIterations',2000,'StepTolerance',1e-6,'Algorithm','sqp');
opt_prob.nonlcon = @(z) collect_constraint(z, problem, robot);
options.screen = 'on';

[zSoln, objVal,exitFlag,output] = fmincon(opt_prob);
save('zInit.mat','zSoln');
%% optimization result
z = reshape(zSoln(1:(end-problem.Nx)),problem.Nx+problem.Nu,problem.Nt-1);
xtraj = [z(1:problem.Nx,:) zSoln(end-(problem.Nx-1):end)];
utraj = z((problem.Nx+1):(problem.Nx+problem.Nu),:);

%% plot
figure(1);
plot(t_hist,xtraj(1,:), 'bo-', 'LineWidth', 2.0);hold on;
plot(t_hist,xtraj(3,:), 'ro-', 'LineWidth', 2.0);
figure(2);
plot(t_hist,xtraj(2,:), 'bo-', 'LineWidth', 2.0); hold on;
plot(t_hist,xtraj(4,:), 'ro-', 'LineWidth', 2.0);

figure(3);
plot(xtraj(1,:), xtraj(2,:), 'o-', 'LineWidth', 2.0);hold on;
plot(xref(1,:), xref(2,:), '--', 'LineWidth', 2.0);hold on;
plot(xtraj(3,:), xtraj(4,:), 'o-', 'LineWidth', 2.0);hold on;
plot(xref(3,:), xref(4,:), '--', 'LineWidth', 2.0);axis equal;

figure(4);
plot(t_hist(1:end-1),utraj(1,:), 'bo-', 'LineWidth', 2.0); hold on;
plot(t_hist(1:end-1),utraj(2,:), 'ro-', 'LineWidth', 2.0);
legend('F', 'Tau');