function X=axxb( e_bh, e_sc )
% e_bh = [ 0.466, 0.003, 0.067  0.667, 0.579, 0.458, 0.099;
% 	  0.451, -0.140, 0.096  0.855, -0.038, 0.388, -0.342;
% 	  0.511, -0.102, 0.150   0.909, 0.099, 0.273, -0.300;
% 	  0.532, 0.062, 0.312 0.953, 0.113, 0.034, 0.278 ];
% e_sc = [ -0.017, 0.030, 0.171  0.347, 0.819, 0.388, -0.243;
% 	-0.035, -0.007, 0.266  0.768, 0.318, 0.530, 0.167;
% 	-0.066, -0.003, 0.262  0.765, 0.464, 0.408, 0.183;
% 	0.034, -0.040, 0.407 0.822 0.513 -0.049 -0.240 ];
% gripper_pick
% tx1 ty1 tz1 qx1 qy1 qz1 qw1
% e_bh = [0.491  0.084 0.301 -0.285 0.958 -0.001 -0.005;
% 0.518  0.070 0.210  0.357 0.925 -0.120 -0.058;
% 0.245  0.271 0.192  0.012 0.877 -0.365  0.314;
% 0.358 -0.071 0.257  0.284 0.863  0.386  0.160;
% 0.553  0.007 0.318 -0.018 0.965 -0.044 -0.258];

% marker
% tx1 ty1 tz1 qx1 qy1 qz1 qw1
% e_sc = [0.050  0.052 0.419 -0.659 0.751 -0.032  0.036;
% -0.043  0.040 0.327 -0.058 0.983 -0.094  0.147;
% -0.057  0.036 0.524 -0.333 0.807 -0.478 -0.096;
%  0.016 -0.034 0.393 -0.116 0.916  0.276 -0.267;
% -0.118  0.002 0.404 -0.423 0.858  0.042  0.288];
% e_bh: a Nx7 matrix that contain N forward kinematics measurements
%       obtained from tf_echo. The format of each row must be 
%       [t_x  t_y t_z  q_x  q_y  q_z  q_w] 
% e_sc: a Nx7 matrix that contain N AR tag measurements obtained from 
%       tf_echo. The format of each row must be 
%       [t_x  t_y  t_z  q_x  q_y q_z  q_w] 
% Return: the 4x4 homogeneous transformation of the hand-eye
%         calibration

% reorder e_bh e_sc --> construct homogenenous matrix --> construct A B -->
% to get alpha beta --> call solveRx solveTx

[N, ~] = size(e_bh);

% 1. convert the quaternion into rotation matrix 
q_ebh = zeros(N,4);                     % to extract and reorder the quaternion
q_esc = zeros(N,4);
for i = 1:N                            % the sequence of qua2rotm & ros is different i.e. [qw qx qy qz] & [qx qy qz qw] , so reoder needed
    q_ebh(i, 2:4) = e_bh(i, 4:6);
    q_ebh(i, 1) = e_bh(i, 7);
    q_esc(i, 2:4) = e_sc(i, 4:6);
    q_esc(i, 1) = e_sc(i, 7);    
end

R_ebh = quat2rotm(q_ebh);
R_esc = quat2rotm(q_esc);

% 2. construct the homogenous 4*4 matrix
g_ebh = zeros(4, 4, N);                 % store homogeneous matrix in 4*4*3 form
g_esc = zeros(4, 4, N);
for i = 1:N
    g_ebh(1:3, 1:3, i) = R_ebh(:, :, i);
    g_esc(1:3, 1:3, i) = R_esc(:, :, i);
    g_ebh(1:3, 4, i) = e_bh(i, 1:3);
    g_esc(1:3, 4, i) = e_sc(i, 1:3);
    g_ebh(4, :, i) = [0 0 0 1];
    g_esc(4, :, i) = [0 0 0 1];
end

A = zeros(4, 4, N);
B = zeros(4, 4, N);
% 3. construct A & B
for i = 1:N-1
    A(:, :, i) = g_ebh(:, :, i) \ g_ebh(:, :, i+1);   % A1 = E1^-1 * E2
    B(:, :, i) = g_esc(:, :, i) / g_esc(:, :, i+1);   % B1 = S1 * S2^-1
end
A(:, :, N) = g_ebh(:, :, 1) \ g_ebh(:, :, N);
B(:, :, N) = g_esc(:, :, 1) / g_esc(:, :, N);

% 4. to get various parameters
RA = zeros(3, 3, N);
RB = zeros(3, 3, N);
TA = zeros(3, N);
TB = zeros(3, N);
for i = 1:N
    RA(:, :, i) = A(1:3, 1:3, i);
    RB(:, :, i) = B(1:3, 1:3, i);
    TA(:, i) = A(1:3, 4, i);
    TB(:, i) = B(1:3, 4, i);
end

alphas = zeros(3, N);
betas = zeros(3, N);
for i = 1:N
    alphas(:, i) = invskew3(logm(RA(:, :, i)));
    betas(:, i) = invskew3(logm(RB(:, :, i)));
end

RX = solveRx( alphas, betas );
TX = solveTx( RA, TA, RB, TB, RX );
X = [RX TX; 0 0 0 1];
end

function res = invskew3(R)
% to calculate the w from w^hat matrix
res = zeros(3, 1);
res(1) = R(3, 2);
res(2) = R(1, 3);
res(3) = R(2, 1);
end