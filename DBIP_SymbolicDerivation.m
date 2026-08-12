clc; clear;

syms etag Kg etam kt Vm km Rm real;
syms theta alpha phi real;
syms thetadot alphadot phidot real;
syms thetaddot alphaddot phiddot real;
syms r l1 l2 Lr L1 L2 real;
syms mr m1 mh m2 Jr J1 J2 g real;
syms tau real;

% Defining torque applied at the base of the rotary arm
% Kt_eff = (etag*Kg*etam*kt)/Rm; --This is the actual formulation but Quanser ommits efficiency terms, to be
% tested later
Kt_eff = (Kg*kt)/Rm;
torque = Kt_eff*(Vm - Kg*km*thetadot);

% --- Tip of arm relative to frame 0
d01 = [Lr*cos(theta), Lr*sin(theta), 0].';

% Distance from frame 1 origin to frame 2 origin
d12 = [0, -L1*sin(alpha), L1*cos(alpha)].';

% Rotation Matrices
R01 = [cos(theta)    -sin(theta)    0;
       sin(theta)     cos(theta)    0;
       0              0             1];

R12 = [1          0           0;
       0   cos(alpha)  -sin(alpha);
       0   sin(alpha)   cos(alpha)];

R23 = [1          0         0;
       0    cos(phi)  -sin(phi);
       0    sin(phi)   cos(phi)];

R02 = R01*R12;
R03 = R01*R12*R23;

% Homogeneous Transformation - Frame 0 to frame 1
H01 = [cos(theta)    -sin(theta)    0            Lr*cos(theta);
       sin(theta)     cos(theta)    0            Lr*sin(theta);
       0              0             1            0;
       0              0             0            1];

H12 = [1              0             0            0;
       0              cos(alpha)    -sin(alpha) -L1*sin(alpha);
       0              sin(alpha)    cos(alpha)   L1*cos(alpha);
       0              0             0            1];

H23 = [1              0             0            0;
       0              cos(phi)      -sin(phi)   -L2*sin(phi);
       0              sin(phi)      cos(phi)     L2*cos(phi);
       0              0             0            1];

% Homogeneous Representation of link 1 cm, hinge, and link 2 cm vectors
P1 = [0, -l1*sin(alpha), l1*cos(alpha), 1].'; % Relative to frame 1
PH = [0, -L1*sin(alpha), L1*cos(alpha), 1].'; % Relative to frame 1
P2 = [0, -l2*sin(phi),   l2*cos(phi),   1].'; % Relative to frame 2

% Positions with respect to frame 0
P01 = H01*P1;
P0H = H01*PH;
P02 = H01*H12*P2;

% 3D Position vectors
r0a = [Lr*cos(theta), Lr*sin(theta), 0].';
r01 = P01(1:3);
r0h = P0H(1:3);
r02 = P02(1:3);

% Finding velocity vectors with respect to generalized coordinates
q = [theta, alpha, phi].';
qdot = [thetadot, alphadot, phidot].';
qddot = [thetaddot, alphaddot, phiddot].';

% Translational velocity vectors relative to world frame x0,y0,z0
r0adot = jacobian(r0a, q)*qdot;
r01dot = jacobian(r01, q)*qdot;
r0hdot = jacobian(r0h, q)*qdot;
r02dot = jacobian(r02, q)*qdot;

% Angular Velocity
w1_12 = [alphadot, 0, 0].'; % Frame 2 relative to frame 1
w2_23 = [phidot, 0, 0].';   % Frame 3 relative to frame 2

% Angular velocity vectors relative to world frame
w0_01 = [0, 0, thetadot].'; 
w0_02 = w0_01 + R01*w1_12;
w0_03 = w0_01 + R01*w1_12 + R02*w2_23;

% Inertia about the center of mass
J_link1 = [(1/12)*m1*L1^2,    0,                 0;
            0,                (1/12)*m1*L1^2,    0;
            0,                0,                 0];

J_link2 = [(1/12)*m2*L2^2,    0,                 0;
            0,                (1/12)*m2*L2^2,    0;
            0,                0,                 0];

I_link1 = R02*J_link1*R02.';
I_link2 = R03*J_link2*R03.';

% Total Potential and Kinetic energies of the system
Vg1 = m1*g*r01(3);
Vgh = mh*g*r0h(3);
Vg2 = m2*g*r02(3);
Vt = simplify(Vg1 + Vgh + Vg2);

Tr_arm = 1/2*Jr*dot(w0_01, w0_01);
Tt_1 = 1/2*m1*dot(r01dot, r01dot);
Tr_1 = 1/2*w0_02.'*I_link1*w0_02;
Tt_h = 1/2*mh*dot(r0hdot, r0hdot);
Tt_2 = 1/2*m2*dot(r02dot, r02dot);
Tr_2 = 1/2*w0_03.'*I_link2*w0_03;

Tt = Tr_arm + Tr_1 + Tt_1 + Tt_h + Tr_2 + Tt_2;

% Generalized forces
% Damping coefficients
Dr = 0.0024;
D1 = 0.0024;
D2 = 0.0024;

Q = [torque - Dr*thetadot;
     -D1*alphadot;
     -D2*phidot];

% Lagrangian
L = simplify(Tt - Vt);

% Partial derivatives
dL_dq = jacobian(L, q).';
dL_dqdot = jacobian(L, qdot).';

% Time derivative of dL_dqdot
ddt_dL_dqdot = jacobian(dL_dqdot, q)*qdot + ...
               jacobian(dL_dqdot, qdot)*qddot;

% Equation of motion
EOM = simplify(ddt_dL_dqdot - dL_dq - Q);
sol = solve(EOM == zeros(3,1), qddot);

% Mass / inertia matrix D(q)
Dmat = simplify(jacobian(EOM, qddot));

% Remaining terms after removing D(q)qddot
remaining = simplify(EOM - Dmat*qddot);
Bmat = -jacobian(remaining, Vm);

% Removing voltage input
remaining_no_input = simplify(subs(remaining, Vm, 0));

% Extract gravity vector
DelP = simplify(subs(remaining_no_input, qdot, zeros(3,1)));

% Damping torques
Fdamp = [Kt_eff*Kg*km*thetadot + Dr*thetadot;
         D1*alphadot;
         D2*phidot];

% Centrifugal and Coriolis forces
Cmat = simplify(remaining_no_input - DelP - Fdamp);

thetaddot_sol = simplify(sol.thetaddot);
alphaddot_sol = simplify(sol.alphaddot);
phiddot_sol = simplify(sol.phiddot);

% Nonlinear state solution
x = [theta, alpha, phi, thetadot, alphadot, phidot].';

xdot = [thetadot;
        alphadot;
        phidot;
        thetaddot_sol;
        alphaddot_sol;
        phiddot_sol];

% Computing the LTI matrices
A_raw = jacobian(xdot, x);
B_raw = jacobian(xdot, Vm);

% Equilibrium point
syms theta_r real;
x_eq = [theta_r;
        0;
        0;
        0;
        0;
        0];

vm_eq = 0;

% Evaluate Jacobians at equilibrium
A_linearized = simplify(subs(A_raw, [x; Vm], [x_eq; vm_eq]));
B_linearized = simplify(subs(B_raw, [x; Vm], [x_eq; vm_eq]));

% Obtain the servo and link angles assuming ideal measurement on all states
C = eye(6);
D = [0, 0, 0, 0, 0, 0].';

% Parameter values
Lr_val = 0.2159;
L1_val = 0.2;
l1_val = 0.1635;
L2_val = 0.3365;
l2_val = 0.1778;

m1_val = 0.097;
mh_val = 0.141;
m2_val = 0.127;

Jr_val = 9.9829E-04;%0.0041;
g_val = 9.81;

etag_val = 1;%0.90;
etam_val = 1;%0.69;
Kg_val = 70;
kt_val = 7.68E-3;
km_val = 7.68E-3;
Rm_val = 2.6;

% Symbolic variables and numerical values
model_params = [Jr, L1, L2, Lr, g, l1, l2, m1, m2, mh, ...
                etag, etam, Kg, kt, km, Rm];

param_values = [Jr_val, L1_val, L2_val, Lr_val, g_val, l1_val, l2_val, ...
                m1_val, m2_val, mh_val, etag_val, etam_val, Kg_val, ...
                kt_val, km_val, Rm_val];

% Saving the file into a MAT-file
save('rotary_double_pendulum_model.mat', ...
     'Dmat', 'Cmat', 'DelP', 'Fdamp', 'Bmat', ...
     'x', 'xdot', 'A_linearized', 'B_linearized', ...
     'C', 'D', 'Vm', 'model_params', 'param_values', ...
     'R01', 'R12', 'd12', 'd01', 'r0a', 'r0h', ...
     'r01', 'r02', '-v7.3');
