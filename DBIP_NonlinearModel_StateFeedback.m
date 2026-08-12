clear;clc;

%Load and prepare symbolic dynamics
syms Dr Dp1 Dp2 real
syms Jr Kg L1 L2 Lr Rm etag etam g km kt l1 l2 m1 m2 mh real;
syms s alpha alphadot phi phidot Vm theta thetadot real;
syms Jr Kg Lp1 Lp2 Lr Rm etag etam g km kt lp1 lp2 Mp1 Mp2 Mh real;
matFile = 'rotary_double_pendulum_model.mat';
load(matFile);
%var is the list of symbolic variables that xdot has
var = [Vm, alpha,alphadot,phi,phidot,thetadot].';

%Substituting numerical values once
x_dot_numeric = subs(xdot, model_params, param_values);

%Converting symbolic acceleration terms to numeric function
%This creates the function handle xdot_func that takes the acceleration terms of x_dot_numeric & replaces
%the symbolic variables with the matching list "var"
xdot_func = matlabFunction(x_dot_numeric(4:6),'Vars',var);

%Simulation setup
tspan = linspace(0,25,1000);
%Initial Condition
x0 = [deg2rad(0); deg2rad(-3); deg2rad(-0.5); 0; 0; 0];% [theta; alpha; phi; thetadot; alphadot; phidot]
%Desired state vector
xd = [0;0;0;0;0;0];

%Obtaining the Linearized Matrices A, B, C & D
A_linearized_numeric = double(subs(A_linearized,model_params,param_values));
B_linearized_numeric = double(subs(B_linearized,model_params,param_values));
if rank(ctrb(A_linearized_numeric,B_linearized_numeric)) == 6 
    fprintf('The system is controllable\n');
else
    fprintf('The system is uncontrollable (rank deficient)\n');
end
%----------------------------------------------------------------------------------
p_des = [-109,-9+1.62j,-9-1.62j,-5,-4,-0.2];
F = place(A_linearized_numeric,B_linearized_numeric,p_des);
disp(F);
%Function wrapper because ode45 takes function odefun of the form odefun(t,x)
ode_handle = @(t,x) odefun(t,x,xd,F,xdot_func);
%Simulating non-linear dynamics
[t,X] = ode45(ode_handle,tspan,x0);
%To compute the voltage input from simulated states
Vm_sim = (F*(xd- X.'));
%Saving data points for 3D visualization
save('Samplerun','X','t');

%Plotting the joint angles
figure(1);

subplot(3,1,1);
p1 = plot(t,rad2deg(X(:,1)),'-b','LineWidth',1.5);
xlabel('Time (s)');
ylabel('Angle \theta (deg)');
title('Rotary Arm');
subplot(3,1,2);
p2 = plot(t,rad2deg(X(:,2)),'-r','LineWidth',1.5);
xlabel('Time (s)');
ylabel('Angle \alpha (deg)');
title('Bottom Pendulum');
subplot(3,1,3);
p3 = plot(t,rad2deg(X(:,3)),'-m','LineWidth',1.5);
xlabel('Time (s)');
ylabel('Angle \phi (deg)');
title('Top Pendulum');
lgd = legend([p1, p2, p3],{'\theta', '\alpha','\phi'});
set(lgd,'Location','northeast');
figure(2);
%Plotting the simulated voltage input
plot(t,Vm_sim,'LineWidth',1.5);
hold on;
yline(10, '--','+10 V limit');
yline(-10,'--','-10 V limit');
hold off;
xlabel('Time (s)');
ylabel('Vm (V)');
title('Control Input Voltage');
grid on;

function xdot = odefun(t, x,xd,F, xdot_func)
    x4 = x(4); x5 = x(5); x6 = x(6);
    Vm_input = double(F*(xd-x));
    % tau_input = -x4;
    %acceleration terms all depend on [Vm, alpha, alphadot, phi, phidot, thetadot]
    accel = xdot_func(Vm_input,x(2),x5,x(3),x6,x4);
    xdot = [x4; x5; x6; accel(1); accel(2); accel(3)];
end




