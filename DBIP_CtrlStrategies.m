clc;clear;close all;
matFile = 'rotary_double_pendulum_model.mat';
load(matFile);
syms x11 x21 x31 x41 x51 x61
syms u11
fprintf('Control Strategy Menu\n');
fprintf('1 = Pole placement with full-state feedback\n');
fprintf('2 = Pole placement with full order observer\n');
fprintf('3 = Pole placement with minimum order observer\n');
fprintf('4 = Full information regulator & tracking\n')
fprintf('5 = Measured information regulator & tracking\n')
fprintf('6 = Structurally Stable Synthesis\n')
control_strategy = input('Select control strategy (1, 2, 3, 4, 5 or 6): ');
%Obtaining the Linearized Matrices A, B, C & D
A_linearized_numeric = double(subs(A_linearized,model_params,param_values));
B_linearized_numeric = double(subs(B_linearized,model_params,param_values));
C_real = [1 0 0 0 0 0;
          0 1 0 0 0 0;
          0 0 1 0 0 0];
D_real = [0;0;0];
%Define an initial condition for the pendulum
theta_i = -4;
alpha_i = 2;
phi_i = -1;
y_o = [deg2rad(theta_i); deg2rad(alpha_i); deg2rad(phi_i)];
x0 = [deg2rad(theta_i); deg2rad(alpha_i); deg2rad(phi_i); 0; 0; 0];% [theta; alpha; phi; thetadot; alphadot; phidot] - Initial condition of linear plant

if control_strategy == 1 %Follows textbook notation style (A+BF)
    if rank(ctrb(A_linearized_numeric,B_linearized_numeric)) == 6 
        fprintf('The system is controllable\n');
    else
        fprintf('The system is uncontrollable (rank deficient)\n');
    end
    p_des = [-109,-9+1.62j,-9-1.62j, -5, -4, -0.2];
    F_standard = place(A_linearized_numeric,B_linearized_numeric,p_des);
    F = -F_standard;
    state_names = {'\theta', '\alpha', '\phi'};
    sim('ModularDBIPSimulation.slx',25);
    for i = 1:3
        subplot(3,1,i);
        plot(t_sim,rad2deg(x_state_ideal(:,i)),'-b', 'LineWidth', 2)
        grid on;
        xlabel('Time (s)');
        ylabel(['$' state_names{i} '(t)$'], ...
           'Interpreter','latex');
        title(['State: $' state_names{i} '(t)$'], ...
          'Interpreter','latex');
    end

elseif control_strategy == 2 %Follows textbook notation style (A+BF) & (A+LC)
    p_des = [-110,-10+1.62j,-10-1.62j, -7, -5, -0.5];
    F = -place(A_linearized_numeric,B_linearized_numeric,p_des);
    Ob_matrix = obsv(A_linearized_numeric,C_real);
    observer_poles = [-121,-122,-123,-124,-125,-126];
    L = -place(A_linearized_numeric.',C_real.',observer_poles).';
    Ao = A_linearized_numeric+L*C_real;
    Bo = [B_linearized_numeric+L*D_real -L];
    Co = eye(6);
    Do = zeros(6,4);
    z_o = [deg2rad(theta_i); deg2rad(alpha_i); deg2rad(phi_i); 0; 0; 0];
    state_names = {'\theta', '\alpha', '\phi', ...
               '\dot{\theta}', '\dot{\alpha}', '\dot{\phi}'};
    sim('ModularDBIPSimulation.slx',25);
    for i = 1:6
        subplot(2,3,i);

        plot(t_sim, rad2deg(x_state_ideal(:,i)), 'k', 'LineWidth', 1.5);
        hold on;
        plot(t_sim, rad2deg(x_state_estimate(:,i)), '--r', 'LineWidth', 1.2);

        grid on;
        xlabel('Time (s)');

        ylabel(['$' state_names{i} '(t)$'], ...
               'Interpreter','latex');

        title(['State comparison: $' state_names{i} '(t)$'], ...
              'Interpreter','latex');

        legend('Ideal','Estimated','Location','best');
    end
        sgtitle('State Feedback with Full Order Observer','FontSize',16,'FontWeight','bold');
elseif control_strategy == 3
    p_des = [-110,-10+1.62j,-10-1.62j, -7, -5, -0.5];
    F = -place(A_linearized_numeric,B_linearized_numeric,p_des);
    min_order_observer_des_p = [-73,-74,-75];
    KerC = null(C_real);
    e1ep = [1 0 0;%basis vectors to extend to X
            0 1 0;
            0 0 1;
            0 0 0;
            0 0 0;
            0 0 0];
    T_inv = [e1ep KerC]; T = T_inv^-1;
    TAT_inv = T*A_linearized_numeric*T_inv;
    TB = T*B_linearized_numeric;        
    CT_inv = C_real*T_inv;                  
    A11 = TAT_inv (1:3, 1:3);
    A12 = TAT_inv (1:3, 4:6);
    A21 = TAT_inv (4:6, 1:3);
    A22 = TAT_inv (4:6, 4:6);
    B1 = TB(1:3,1);
    B2 = TB(4:6,1);
    C1 = CT_inv(1:3,1:3);
    L = -place(A22.',A12.',min_order_observer_des_p).';
    M = A22 + L*A12; 
    N = B2+L*B1; 
    R = zeros(6,1);
    P = (A21+L*A11-A22*L-L*A12*L)*C1^-1;
    zero_matrix = zeros(3,3);
    identity_3 = eye(3);
    Q = T_inv*[zero_matrix;identity_3]; 
    S = T_inv*[C1^-1;-L*C1^-1];
    Ao = M;
    Bo = [N P];
    Co = Q;
    Do = [R S];
    z_o = L*(C1^-1)*y_o; %For the real hardware, I believe a good initial condition is the zero column vector (6x1)
    state_names = {'\theta', '\alpha', '\phi', ...
               '\dot{\theta}', '\dot{\alpha}', '\dot{\phi}'};
    sim('ModularDBIPSimulation.slx',25);
    for i = 1:6
        subplot(2,3,i);
        if i<=3
        % Measured position states
        plot(t_sim, rad2deg(squeeze(y_state_measured(i,1,:))),'k', 'LineWidth', 1.5);

        legend('Measured','Location','best');

        else
        % True and estimated velocity states
        plot(t_sim, rad2deg(x_state_ideal(:,i)), 'k', 'LineWidth', 1.5);
        hold on;
        plot(t_sim, rad2deg(x_min_order_state_estimate(:,i)), '--r', 'LineWidth', 1.2);
        hold off;
        legend('Ideal','Estimated','Location','best');
        end
        grid on;
        xlabel('Time (s)');

        ylabel(['$' state_names{i} '(t)$'], ...
               'Interpreter','latex');

        title(['State comparison: $' state_names{i} '(t)$'], ...
              'Interpreter','latex');
    end
        sgtitle('State Feedback with Minimum-Order Observer','FontSize',16,'FontWeight','bold');
elseif control_strategy == 4
    A = [
    0         0         0     1.0000         0         0         0;
    0         0         0         0     1.0000         0         0;
    0         0         0         0         0     1.0000         0;
    0    493.7629   -3.5865  -74.8161   -1.7518    1.7906         0;
    0    597.3631  -23.7073  -82.8864   -2.1875    2.4444         0;
    0   -610.6139   83.3820   84.7249    2.4444   -3.3478         0;
    0         0         0         0         0         0         0];
    A1 = [0         0         0    1.0000         0         0;
          0         0         0         0    1.0000         0;
          0         0         0         0         0    1.0000;
          0  493.7629   -3.5865  -74.8161   -1.7518    1.7906;
          0  597.3631  -23.7073  -82.8864   -2.1875    2.4444;
          0 -610.6139   83.3820   84.7249    2.4444   -3.3478];
    A3 = [0;0;0;0;0;0];
    A2 = 0;
   B = [0;0;0;136.2256;150.9200;-154.2677;0]; 
   B1 = [0;0;0;136.2256;150.9200;-154.2677];
   C_7 = eye(7);
   D_7 = [-1,0,0,0,0,0,1];
   D1 = [-1,0,0,0,0,0];
   D2 = 1;
   p_des = [-110,-10+1.62j,-10-1.62j, -7, -5, -0.5];
   F1 = -place(A_linearized_numeric,B_linearized_numeric,p_des);
   %------Solving for the Regulator Equations
   X = [x11;x21;x31;x41;x51;x61];
   U = u11;
   eq1 = A1*X - X*A2 + A3 + B1*U;
   eq2 = D1*X + D2;
   sol = solve([eq1;eq2] == 0,[x11 x21 x31 x41 x51 x61 u11]);
   X_sol = double([sol.x11;sol.x21;sol.x31;sol.x41;sol.x51;sol.x61]);
   U_sol = double(sol.u11);
   fprintf('This is the X solution: \n');
   disp(X_sol);
   fprintf('This is the U solution: \n');
   disp(U_sol);
   fprintf('The updated feedback controller [F1 F2]: \n');
   format short g;
   F2 = U_sol-F1*X_sol;
   F = [F1 F2];
   disp(F);
   theta_desired = input('Input desired Arm Reference position (deg):');
   x0 = [deg2rad(theta_i); deg2rad(alpha_i); deg2rad(phi_i); 0; 0; 0];
   x2_ref = deg2rad(theta_desired);
   state_names = {'\theta', '\alpha', '\phi'};
   sim('ModularDBIPSimulation.slx',25);
    for i = 1:3
        subplot(3,1,i);
        plot(t_sim,rad2deg(x_state_ideal(:,i)),'-b', 'LineWidth', 2)
        grid on;
        xlabel('Time (s)');
        ylabel(['$' state_names{i} '(t)$'], ...
           'Interpreter','latex');
        title(['State: $' state_names{i} '(t)$'], ...
          'Interpreter','latex');
    end
elseif control_strategy == 5 %Measured information regulator & tracking
    A = [
    0         0         0     1.0000         0         0         0;
    0         0         0         0     1.0000         0         0;
    0         0         0         0         0     1.0000         0;
    0    493.7629   -3.5865  -74.8161   -1.7518    1.7906         0;
    0    597.3631  -23.7073  -82.8864   -2.1875    2.4444         0;
    0   -610.6139   83.3820   84.7249    2.4444   -3.3478         0;
    0         0         0         0         0         0         0];
    A1 = [0         0         0    1.0000         0         0;
          0         0         0         0    1.0000         0;
          0         0         0         0         0    1.0000;
          0  493.7629   -3.5865  -74.8161   -1.7518    1.7906;
          0  597.3631  -23.7073  -82.8864   -2.1875    2.4444;
          0 -610.6139   83.3820   84.7249    2.4444   -3.3478];
    A3 = [0;0;0;0;0;0];
    A2 = 0;
    B = [0;0;0;136.2256;150.9200;-154.2677;0];
    B1 = [0;0;0;136.2256;150.9200;-154.2677];
    C_7 = [1 0 0 0 0 0 0;
           0 1 0 0 0 0 0;
           0 0 1 0 0 0 0;
           -1 0 0 0 0 0 1];
    D_7 = [-1,0,0,0,0,0,1];
    D1 = [-1,0,0,0,0,0];
    D2 = 1;
    X = [x11;x21;x31;x41;x51;x61];
   U = u11;
   eq1 = A1*X - X*A2 + A3 + B1*U;
   eq2 = D1*X + D2;
   sol = solve([eq1;eq2] == 0,[x11 x21 x31 x41 x51 x61 u11]);
   X_sol = double([sol.x11;sol.x21;sol.x31;sol.x41;sol.x51;sol.x61]);
   U_sol = double(sol.u11);
   %Using the same controller designed for the full information case
   p_des = [-110,-10+1.62j,-10-1.62j, -6, -4, -0.5];
   F1 = -place(A_linearized_numeric,B_linearized_numeric,p_des);
   F2 = U_sol-F1*X_sol;
   F = [F1 F2];
   %Taking L values so that A + LC is stable
   obvs_poles_des = [-161,-162,-163,-164,-165,-166,-167];
   L = -place(A.',C_7.',obvs_poles_des).';
   disp(L);
   %Initial condition of the controller (xc)
   Ac = A+B*F+L*C_7;
   Bc = -L;
   Cc = F;
   Dc = zeros(1,4);
   theta_ref = input('Enter the desired reference theta (deg):');
   xc_o = [deg2rad(theta_i); deg2rad(alpha_i); deg2rad(phi_i); 0; 0;0;deg2rad(theta_ref)];
   state_names = {'\theta', '\alpha', '\phi'};
   sim('ModularDBIPSimulation.slx',25);
    for i = 1:3
        subplot(3,1,i);
        plot(t_sim,rad2deg(x_state_ideal(:,i)),'-b', 'LineWidth', 2)
        grid on;
        xlabel('Time (s)');
        ylabel(['$' state_names{i} '(t)$'], ...
           'Interpreter','latex');
        title(['State: $' state_names{i} '(t)$'], ...
          'Interpreter','latex');
    end
elseif control_strategy == 6 %Structurally Stable Synthesis
Ac1 = 0;
Bc1 = [0 0 0 1];
Aa = [
     0         0         0     1.0000         0         0         0;
     0         0         0          0     1.0000         0         0;
     0         0         0          0          0     1.0000         0;
     0   493.7629   -3.5865   -74.8161   -1.7518    1.7906         0;
     0   597.3631  -23.7073   -82.8864   -2.1875    2.4444         0;
     0  -610.6139   83.3820    84.7249    2.4444   -3.3478         0;
    -1         0         0          0          0         0         0];
A1 = Aa(1:6,1:6);
Ba = [
      0;
      0;
      0;
    136.2256;
    150.9200;
   -154.2677;
      0];
B1 = Ba(1:6,:);
C1 = [1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 1 0 0 0;
     -1 0 0 0 0 0];
%Computing the controller
p_controller = [-110, -9.85 + 1.62i, -9.85 - 1.62i, -6, -4.2, -0.5 + 0.27i, -0.5 - 0.27i];
Fa = -place(Aa,Ba,p_controller);
Fa1 = Fa(:,1:6);
Fa2 = Fa(:,7);
%Constructing the observer
p_observer = [-147,-149,-151,-153,-155,-157];
L1 = -place(A1.',C1.',p_observer).';
%Constructing the matrices
Ac2 = A1 + B1*Fa1 + L1*C1;
Ac3 = B1*Fa2;
Bc2 = -L1;
Cc1 = Fa2;
Cc2 = Fa1;
Dc = zeros(1,4);
%Constructing the controller
%Controller inputs are [theta;alpha;phi;e] where e = r - theta
Ac = [Ac1 zeros(1,6);Ac3 Ac2];
Bc = [Bc1;Bc2];
Cc = [Cc1 Cc2];
controller = ss(Ac,Bc,Cc,Dc);
theta_ref = input('Enter the desired reference theta (deg):');
xc_o = [deg2rad(theta_i); deg2rad(alpha_i); deg2rad(phi_i); 0; 0;0;deg2rad(theta_ref)];
state_names = {'\theta', '\alpha', '\phi'};
sim('ModularDBIPSimulation.slx',25);
    for i = 1:3
        subplot(3,1,i);
        plot(t_sim,rad2deg(x_state_ideal(:,i)),'-b', 'LineWidth', 2)
        grid on;
        xlabel('Time (s)');
        ylabel(['$' state_names{i} '(t)$'], ...
           'Interpreter','latex');
        title(['State: $' state_names{i} '(t)$'], ...
          'Interpreter','latex');
    end
end
