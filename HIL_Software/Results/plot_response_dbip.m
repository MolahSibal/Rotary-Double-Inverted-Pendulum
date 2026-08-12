%% PLOT_RESPONSE
% Plots the response of variables in the Matlab workspace.
%
%% Load sample data from MAT files
% Comment these lines out if you want to use the data recently stored.
% load('data_theta.mat');
% load('data_alpha.mat');
% load('data_phi.mat');
% load('data_vm.mat');
%
%% Setup variables
% Load from variables set in workspace after running a Simulink model or
% from the previously saved response saved in the MAT files above.
t = data_theta(:,1);          % time (s)
th_d = data_theta(:,2);     % setpoint
th = data_theta(:,3);       % measured process variable
alpha = data_alpha(:,2);    % bottom / short pendulum angle
phi = data_phi(:,2);        % medium / top pendulum angle
u = data_vm(:,2);           % input signal, srv02
%
%% Plot response
subplot(3,2,1:2);
plot(t,th_d,'b:',t,th,'r-','linewidth',2);
xlabel('time (s)');
ylabel('\theta (deg)');
%
subplot(3,2,3);
plot(t,alpha,'r-','linewidth',2);
xlabel('time (s)');
ylabel('\alpha (deg)');
%
subplot(3,2,4);
plot(t,phi,'r-','linewidth',2);
xlabel('time (s)');
ylabel('\phi (deg)');
%
subplot(3,2,5:6);
plot(t,u,'r-','linewidth',2);
ylabel('V_{m} (V)');

% Find specifications
% c_ts = 0.04;
% tol_p = 0;
% [ tp, ts, e_ss, PO ] = meas_step_rsp_specs( t, th_d, th, c_ts, tol_p );
% Display results
% e_ss
% ts
% PO
% max_alpha = max(abs(alpha))
% max_phi = max(abs(phi))
