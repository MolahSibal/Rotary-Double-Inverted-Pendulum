%% Rotary Double Inverted Pendulum
%
% Sets the necessary parameters to run the Rotary Double Inverted Pendulum 
% laboratory using the "s_dbip" and "q_dbip" Simulink diagrams.
% 
% Copyright (C) 2012 Quanser Consulting Inc.
%
clc;clear;
%
syms x11 x21 x31 x41 x51 x61 u11 real;
%% SRV02 Configuration
% External Gear Configuration: set to 'HIGH' or 'LOW'
EXT_GEAR_CONFIG = 'HIGH';
% Encoder Type: set to 'E' or 'EHR'
ENCODER_TYPE = 'E';
% Is SRV02 equipped with Tachometer? (i.e. option T): set to 'YES' or 'NO'
TACH_OPTION = 'YES';
% Is SRV02 equipped with slip-ringt (i.e. option ETS): set to 'YES' or 'NO'
ETS_OPTION = 'NO';
% Type of Load: set to 'NONE', 'DISC', or 'BAR'
LOAD_TYPE = 'NONE';
% Amplifier Gain used: 
% VoltPAQ-X1 users: set to K_AMP to 1 and Gain switch on amplifier to 1
% VoltPAQ-X2 users: set to K_AMP 3
K_AMP = 1;
% Power Amplifier Type: set to 'VoltPAQ', 'UPM_1503', 'UPM_2405', or 'Q3'
AMP_TYPE = 'VoltPAQ';
% Digital-to-Analog Maximum Voltage (V)
VMAX_DAC = 10;
%
%% Double-Pendulum Configuration
% ROTPEN Option: 'ROTPEN' or 'ROTPEN-E'
ROTPEN_OPTION = 'ROTPEN-E';
% Define rotary arm attached to SRV02 load gear.
SRV02_ARM = 'ROTARY_ARM';
% Length of pendulums.
% Bottom 7-inch short link.
PEND_1_TYPE = 'SHORT_7IN';
% Upper 12-inch medium link.
PEND_2_TYPE = 'MEDIUM_12IN';
%
%% Safety Watchdog
% Safety watchdog on the SRV02 arm angle: ON = 1, OFF = 0 
THETA_LIM_ENABLE = 1;       % safety watchdog turned ON
% THETA_LIM_ENABLE = 0;      % safety watchdog turned OFF
% Safety Limits on the SRV02 arm angle (deg)
THETA_MAX = 90;            % pendulum angle maximum safety position (deg)
THETA_MIN = - THETA_MAX;   % pendulum angle minimum safety position (deg)
%
% Safety watchdog on pendulum 1 angle: ON = 1, OFF = 0 
ALPHA_LIM_ENABLE = 1;       % safety watchdog turned ON
% ALPHA_LIM_ENABLE = 0;      % safety watchdog turned OFF
% Safety Limits on the pendulum 1 angle (deg)
ALPHA_MAX = 45;            % pendulum angle maximum safety position (deg)
ALPHA_MIN = - ALPHA_MAX;   % pendulum angle minimum safety position (deg)
%
% Safety watchdog on pendulum 2 angle: ON = 1, OFF = 0 
GAMMA_LIM_ENABLE = 1;       % safety watchdog turned ON
%ALPHA_LIM_ENABLE = 0;      % safety watchdog turned OFF
% Safety Limits on pendulum 2 angle (deg)
%global ALPHA_MAX ALPHA_MIN
GAMMA_MAX = 25;            % pendulum angle maximum safety position (deg)
GAMMA_MIN = - GAMMA_MAX;   % pendulum angle minimum safety position (deg)
%
%% System Parameters
% Sets model variables according to the user-defined SRV02 configuration
[ Rm, kt, km, Kg, eta_g, Beq, Jm, Jeq, eta_m, K_POT, K_TACH, K_ENC, VMAX_AMP, IMAX_AMP ] = config_srv02( EXT_GEAR_CONFIG, ENCODER_TYPE, TACH_OPTION, AMP_TYPE, LOAD_TYPE );
% Load rotary arm parameters
[ g, Mr, Lr, lr, Jr, Dr ] = config_sp( 'ROTARY_ARM', 'ROTPEN-E' );
% Load medium 12-inch pendulum parameters
[ g, Mp2, Lp2, lp2, Jp2, Dp2 ] = config_sp( 'MEDIUM_12IN', 'ROTPEN-E' );
% Load short 7-inch pendulum parameters
[ g, Mp1, Lp1, lp1, Jp1, Dp1 ] = config_sp( 'SHORT_7IN', 'ROTPEN-E' );
% Mass of hinge between pendulum 1 and 2 (kg)
Mh = 0.141;
% Set Open-loop state-space model of rotary double-inverted pendulum
DBIP_ABCD_eqns;
% Initial condition used in state-space model for simulation.
X0 = [-5, 1, -0.5, 0, 0, 0] * pi / 180;
%
%% Filter Parameters
% SRV02 High-pass filter in PD control used to compute velocity
% Cutoff frequency (rad/s)
wcf_1 = 2 * pi * 50.0;
% Damping ratio
zetaf_1 = 0.9;
% Pendulum High-pass filter in PD control used to compute velocity
% Cutoff frequency (rad/s)
wcf_2 = 2 * pi * 15.0;
% Damping ratio
zetaf_2 = 0.9;
%
%% Augment the state-space system to include an integrator: zeta_dot = x
A_linearized_numeric = [0         0         0    1.0000         0         0;
     0         0         0         0    1.0000         0;
     0         0         0         0         0    1.0000;
     0  493.7629   -3.5865  -74.8161   -1.7518    1.7906;
     0  597.3631  -23.7073  -82.8864   -2.1875    2.4444;
     0 -610.6139   83.3820   84.7249    2.4444   -3.3478];
B_linearized_numeric = [    0;
         0;
         0;
  136.2256;
  150.9200;
 -154.2677];
Ai = A; Bi = B;
Ai(7,1) = 1; Ai(7,7) = 0; Bi(7,1) = 0;
C_real = [1 0 0 0 0 0;
          0 1 0 0 0 0;
          0 0 1 0 0 0];
D_real = [0;0;0];
%
%% Controller Design         
% Integrator anti-windup parameters
INT_WDUP_MAX = 5;                % maximum integrator output voltage (V)
INT_WDUP_MIN = -5;               % minimum integrator output voltage (V)
Q = diag([ 1 1 1 5 1 1 0.5]);
R = 30;        
%%-------------------Control Methodologies
fprintf('Control Strategy Menu\n');
fprintf('1 = Pole placement with full-state feedback\n');
fprintf('2 = Pole placement with full order observer\n');
fprintf('3 = Pole placement with minimum order observer\n');
fprintf('4 = Full information regulator & tracking\n')
fprintf('5 = Measured information regulator & tracking\n')
fprintf('6 = Structurally Stable Synthesis\n')
control_strategy = input('Select control strategy (1, 2, 3, 4, 5 or 6): ');

if control_strategy == 1 %Pole Placement with full state feedback %good
    if rank(ctrb(A_linearized_numeric,B_linearized_numeric)) == 6 
        fprintf('The system is controllable\n');
    else
        fprintf('The system is uncontrollable (rank deficient)\n');
    end
    p_des = [-109,-9+1.62j,-9-1.62j, -5, -4, -0.2];
    F_standard = place(A_linearized_numeric,B_linearized_numeric,p_des);
    F = -F_standard;

elseif control_strategy == 2 %Pole placement with full order observer %good
    p_des = [-110,-10+1.62j,-10-1.62j, -7, -5, -0.5]; %A bit  aggresive
    %p_des = [-109,-9+1.62j,-9-1.62j, -5, -4, -0.5];
    F = -place(A_linearized_numeric,B_linearized_numeric,p_des);
    Ob_matrix = obsv(A_linearized_numeric,C_real);
    observer_poles = [-121,-122,-123,-124,-125,-126];
    L = -place(A_linearized_numeric.',C_real.',observer_poles).';
    Ao = A_linearized_numeric+L*C_real;
    Bo = [B_linearized_numeric+L*D_real -L];
    Co = eye(6);
    Do = zeros(6,4);
    z_o = [0; 0; 0; 0; 0; 0];

elseif control_strategy == 3 %Pole placement with minimum order observer 
    p_des = [-110,-10+1.62j,-10-1.62j, -7, -5, -0.5];%Good
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
    y_o = [0;0;0];
    z_o = L*(C1^-1)*y_o; 

elseif control_strategy == 4 %Full information regulator & tracking %Works but terrible performance
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
   p_des = [-110,-10+1.62j,-10-1.62j, -6, -5, -0.5];
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

elseif control_strategy == 5 %Measured Information regulator & tracking %Not great Performance 
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
   xc_0 = zeros(7,1);

elseif control_strategy == 6 %Structurally Stable Synthesis %Good performance
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
  xc_o = zeros(7,1);
end