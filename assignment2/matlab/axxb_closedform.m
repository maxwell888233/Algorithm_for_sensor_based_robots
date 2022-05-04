function X=axxb_closedform( e_bh, e_sc )
% e_bh: a 3x7 matrix that contain 3 forward kinematics measurements
% obtained from tf_echo. The format of each row must be
% [𝑡𝑡𝑥𝑥 𝑡𝑡𝑦𝑦 𝑡𝑡𝑧𝑧 𝑞𝑞𝑥𝑥 𝑞𝑞𝑦𝑦 𝑞𝑞𝑧𝑧 𝑞𝑞𝑤𝑤]
% e_sc: a 3x7 matrix that contain 3 AR tag measurements obtained from
% tf_echo. The format of each row must be [𝑡𝑡𝑥𝑥 𝑡𝑡𝑦𝑦 𝑡𝑡𝑧𝑧 𝑞𝑞𝑥𝑥 𝑞𝑞𝑦𝑦 𝑞𝑞𝑧𝑧 𝑞𝑞𝑤𝑤]
% return: the 4x4 homogeneous transformation of the hand-eye
% calibration

%% test
% [e_bh, e_sc, ~] = generatedata(3)

%% construct the A & B based on e_bh & e_sc
% 1. convert the quaternion into rotation matrix 
q_ebh = zeros(3,4);                     % to extract and reorder the quaternion
q_esc = zeros(3,4);
for i = 1:3                             % the sequence of qua2rotm & ros is different i.e. [qw qx qy qz] & [qx qy qz qw] , so reoder needed
    q_ebh(i, 2:4) = e_bh(i, 4:6);
    q_ebh(i, 1) = e_bh(i, 7);
    q_esc(i, 2:4) = e_sc(i, 4:6);
    q_esc(i, 1) = e_sc(i, 7);    
end

R_ebh = zeros(3,3,3);                   % convert qua 2 rotm
R_esc = zeros(3,3,3);
for i = 1:3
    R_ebh(:, :, i) = quat2rotm(q_ebh(i, :));
    R_esc(:, :, i) = quat2rotm(q_esc(i, :));
end

% 2. construct the homogenous 4*4 matrix
g_ebh = zeros(4, 4, 3);                 % store homogeneous matrix in 4*4*3 form
g_esc = zeros(4, 4, 3);
for i = 1:3
    g_ebh(1:3, 1:3, i) = R_ebh(:, :, i);
    g_esc(1:3, 1:3, i) = R_esc(:, :, i);
    g_ebh(1:3, 4, i) = e_bh(i, 1:3);
    g_esc(1:3, 4, i) = e_sc(i, 1:3);
    g_ebh(4, :, i) = [0 0 0 1];
    g_esc(4, :, i) = [0 0 0 1];
end

% 3. construct A & B
A1 = g_ebh(:, :, 1) \ g_ebh(:, :, 2);   % A1 = E1^-1 * E2
A2 = g_ebh(:, :, 2) \ g_ebh(:, :, 3);
B1 = g_esc(:, :, 1) / g_esc(:, :, 2);   % B1 = S1 * S2^-1
B2 = g_esc(:, :, 2) / g_esc(:, :, 3);


%% solve for rotation part
% 1. to get various parameters
RA1 = A1(1:3, 1:3);
RA2 = A2(1:3, 1:3);
RB1 = B1(1:3, 1:3);
RB2 = B2(1:3, 1:3);

TA1 = A1(1:3, 4);
TA2 = A2(1:3, 4);
TB1 = B1(1:3, 4);
TB2 = B2(1:3, 4);

alpha1 = invskew3(logm(RA1));
alpha2 = invskew3(logm(RA2));
alpha3 = cross(alpha1, alpha2);
Alpha = [alpha1 alpha2 alpha3];

beta1 = invskew3(logm(RB1));
beta2 = invskew3(logm(RB2));
beta3 = cross(beta1, beta2);
Beta = [beta1 beta2 beta3];

% 2. 
RX = Alpha / Beta;

%% solve for translation part
left_matrix = [RA1 - eye(3);
               RA2 - eye(3)];
right_matrix = [RX * TB1 - TA1;
                RX * TB2 - TA2];
TX = left_matrix \ right_matrix;

X = [RX TX; 0 0 0 1];

end

function res = invskew3(R)
% to calculate the w from w^hat matrix
res = zeros(3, 1);
res(1) = R(3, 2);
res(2) = R(1, 3);
res(3) = R(2, 1);
end