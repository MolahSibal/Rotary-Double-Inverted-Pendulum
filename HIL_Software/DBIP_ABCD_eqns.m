JT = -2*Mp1*lp1*Lr^2*Mh*Lp1+Mh*Lp1^2*Mp1*Lr^2+Mp1*lp1^2*Mh*Lr^2+Mh*Lp1^2*Jr+Mp1*lp1^2*Jr;

A = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];
    
 
A( 4, 2 ) = Lr*g*(Mp1^2*lp1^2+2*Mp1*lp1*Mh*Lp1+Mh*Lp1^2*Mp2+Mp1*lp1^2*Mp2+Mh^2*Lp1^2)/JT;
A( 4, 3 ) = -Mp1*lp1*Lr*Mp2*g*(-lp1+Lp1)/JT;
A( 4, 4 ) = -Dr*(Mh*Lp1^2+Mp1*lp1^2)/JT;
A( 4, 5 ) = -Lr*Dp1*(Mh*Lp1+Mp1*lp1)/JT;
A( 4, 6 ) = Lr*Dp2*(-Mp1*lp1^2+Mp1*lp1*lp2+Mh*Lp1*lp2+Mp1*lp1*Lp1)/lp2/JT;
A( 5, 1 ) = 0;
A( 5, 2 ) = g*(Lr^2*lp1*Mp1*Mp2+Lp1*Lr^2*Mh*Mp2+Lp1*Lr^2*Mh^2+Lr^2*lp1*Mp1^2+Jr*Lp1*Mh+Jr*lp1*Mp1+Lp1*Lr^2*Mh*Mp1+Lr^2*lp1*Mh*Mp1)/JT;
A( 5, 3 ) = -Mp2*g*(-Lr^2*lp1*Mp1+Lp1*Lr^2*Mp1+Jr*Lp1)/JT;
A( 5, 4 ) = -Dr*Lr*(Mh*Lp1+Mp1*lp1)/JT;
A( 5, 5 ) = -Dp1*(Jr+Mp1*Lr^2+Mh*Lr^2)/JT;
A( 5, 6 ) = Dp2*(Jr*lp2+Jr*Lp1-Lr^2*lp1*Mp1+Lp1*Lr^2*Mp1+Lr^2*lp2*Mh+Lr^2*lp2*Mp1)/lp2/JT;
A( 6, 1 ) = 0;
A( 6, 2 ) = -g/lp2*(Lp1*Lr^2*lp1*Mp1^2+Lp1*Lr^2*lp2*Mh*Mp1+Lr^2*lp1*lp2*Mh*Mp1+Lp1*Lr^2*lp2*Mh^2+Lr^2*lp1*lp2*Mp1^2-Mp1*lp1^2*Lr^2*Mp2+Lr^2*lp1*lp2*Mp1*Mp2+Lp1*Lr^2*lp2*Mh*Mp2+Jr*Lp1*lp2*Mh+Jr*lp1*lp2*Mp1-Mp1*lp1^2*Jr+Lp1*Jr*lp1*Mp1-Mp1*lp1^2*Mh*Lr^2+Lr^2*Lp1*Mp2*Mp1*lp1+Mp1*lp1*Lr^2*Mh*Lp1-Lr^2*Mp1^2*lp1^2)/JT;
A( 6, 3 ) = g/lp2*(Jr*Lp1*lp2*Mp2+Jr*Lp1^2*Mp2+Lp1^2*Lr^2*Mp1*Mp2+Mp1*lp1^2*Lr^2*Mp2+Lp1*Lr^2*lp2*Mp1*Mp2-Lr^2*lp1*lp2*Mp1*Mp2+Mp1*lp1^2*Jr+Mh*Lp1^2*Jr+Mh*Lp1^2*Mp1*Lr^2+Mp1*lp1^2*Mh*Lr^2-2*Lr^2*Lp1*Mp2*Mp1*lp1-2*Mp1*lp1*Lr^2*Mh*Lp1)/JT;
A( 6, 4 ) = Dr*Lr/lp2*(-Mp1*lp1^2+Mp1*lp1*lp2+Mh*Lp1*lp2+Mp1*lp1*Lp1)/JT;
A( 6, 5 ) = Dp1/lp2*(Jr*lp2+Jr*Lp1-Lr^2*lp1*Mp1+Lp1*Lr^2*Mp1+Lr^2*lp2*Mh+Lr^2*lp2*Mp1)/JT;
A( 6, 6 ) = -Dp2*(-2*Lr^2*Lp1*Mp2*Mp1*lp1-2*Mp1*lp1*Lr^2*Mh*Lp1+2*Lp1*Lr^2*lp2*Mp1*Mp2+Jr*Lp1^2*Mp2+Mp2*Jr*lp2^2+Mh*Lp1^2*Jr+Mp1*lp1^2*Jr+2*Jr*Lp1*lp2*Mp2-2*Lr^2*lp1*lp2*Mp1*Mp2+Mh*Lp1^2*Mp1*Lr^2+Mp1*lp1^2*Mh*Lr^2+Mp1*lp1^2*Lr^2*Mp2+Lp1^2*Lr^2*Mp1*Mp2+Mp2*Lr^2*lp2^2*Mh+Mp2*Lr^2*lp2^2*Mp1)/lp2^2/JT/Mp2;

B = [0;
    0;
    0;
    (Mh*Lp1^2+Mp1*lp1^2)/JT;
    Lr*(Mh*Lp1+Mp1*lp1)/JT;
    -Lr/lp2*(-Mp1*lp1^2+Mp1*lp1*lp2+Mh*Lp1*lp2+Mp1*lp1*Lp1)/JT];

C = eye(6,6);

D = zeros(6,1);

% Add actuator dynamics
B = Kg * kt * B / Rm;
A(4,4) = A(4,4) - Kg*km*B(4);
A(5,4) = A(5,4) - Kg*km*B(5);
A(6,4) = A(6,4) - Kg*km*B(6);
