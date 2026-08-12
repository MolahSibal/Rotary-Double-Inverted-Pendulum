clc;clear;close all;
%Load and prepare symbolic dynamics
syms alpha alphadot phi phidot tau theta thetadot real;
syms r l1 l2 Lr L1 L2 real;
matFile = 'rotary_double_pendulum_model.mat';
simFile = 'Samplerun.mat';
load(matFile);
load('Samplerun.mat');
Lr_val = param_values(4); L1_val = param_values(2) ; l1_val = param_values(6);l2_val = param_values(7);

% ----------------------------- 3D Animation

% Approximate full length of link 2
L2_val = 0.3556;

% Physical tip of link 2 written relative to frame 2
p2_tip = [0;
         -L2*sin(phi);
          L2*cos(phi)];

% Position of link 2 tip in frame 0
r03 = simplify(R01*(R12*p2_tip + d12) + d01);

position_function = matlabFunction( ...
    [r0a, r0h, r03, r01, r02], ...
    'Vars', {[theta; alpha; phi], Lr, L1, L2, l1, l2});

figure(2)
clf

ax2 = axes;
axis(ax2,'equal')
grid(ax2,'on')
hold(ax2,'on')
view(ax2,3)

xlabel(ax2,'x_0 [m]')
ylabel(ax2,'y_0 [m]')
zlabel(ax2,'z_0 [m]')
title(ax2,'Rotary Double Inverted Pendulum')

maximum_reach = Lr_val + L1_val + L2_val;

xlim([-maximum_reach maximum_reach])
ylim([-maximum_reach maximum_reach])
zlim([-maximum_reach maximum_reach])

arm_plot = plot3(nan,nan,nan,'LineWidth',4);
link1_plot = plot3(nan,nan,nan,'LineWidth',4);
link2_plot = plot3(nan,nan,nan,'LineWidth',4);

motor_plot = plot3(0,0,0,'o', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','k');

arm_joint_plot = plot3(nan,nan,nan,'o', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','k');

hinge_plot = plot3(nan,nan,nan,'o', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','k');

cm1_plot = plot3(nan,nan,nan,'o', ...
    'MarkerSize',7, ...
    'MarkerFaceColor','r');

cm2_plot = plot3(nan,nan,nan,'o', ...
    'MarkerSize',7, ...
    'MarkerFaceColor','r');

O0 = [0;0;0];

animation_step = 1;

for k = 1:animation_step:length(t)-1

    current_angles = X(k,1:3).';

    positions = double(position_function( ...
        current_angles, ...
        Lr_val, ...
        L1_val, ...
        L2_val, ...
        l1_val, ...
        l2_val));

    O1  = positions(:,1);    % Rotary arm tip
    O2  = positions(:,2);    % Hinge
    O3  = positions(:,3);    % End of link 2
    CM1 = positions(:,4);    % Link 1 center of mass
    CM2 = positions(:,5);    % Link 2 center of mass

    set(arm_plot, ...
        'XData',[O0(1),O1(1)], ...
        'YData',[O0(2),O1(2)], ...
        'ZData',[O0(3),O1(3)]);

    set(link1_plot, ...
        'XData',[O1(1),O2(1)], ...
        'YData',[O1(2),O2(2)], ...
        'ZData',[O1(3),O2(3)]);

    set(link2_plot, ...
        'XData',[O2(1),O3(1)], ...
        'YData',[O2(2),O3(2)], ...
        'ZData',[O2(3),O3(3)]);

    set(arm_joint_plot, ...
        'XData',O1(1), ...
        'YData',O1(2), ...
        'ZData',O1(3));

    set(hinge_plot, ...
        'XData',O2(1), ...
        'YData',O2(2), ...
        'ZData',O2(3));

    set(cm1_plot, ...
        'XData',CM1(1), ...
        'YData',CM1(2), ...
        'ZData',CM1(3));

    set(cm2_plot, ...
        'XData',CM2(1), ...
        'YData',CM2(2), ...
        'ZData',CM2(3));

    title(ax2, sprintf('Rotary Double Inverted Pendulum — t = %.2f s', t(k)))

    drawnow
   pause(t(k+1)-t(k))
end
