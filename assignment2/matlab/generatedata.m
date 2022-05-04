% Generate synthetic data to test hand-eye calibration.
% e_bh and e_sc are Nx7 matrices that represent N E_bh and N E_sc
% transformations. Each row is of the form [ tx ty tz qx qy qz qw ]
% were tx, ty and tz denote a translation and qx, qy, qz, qw a
% quaternion.
% X is a randomly generated hand-eye transformation
function [e_bh, e_sc, X] = generatedata(N)
% Assume we know the transformation between the base and the
% checkerboard
E_bc = [ eye(3) [ 1; 0; 0 ]; 0 0 0 1 ];
% Create a random X for generating the data
X = randSE3();
% X = [eye(3), [1;1;1];0 0 0 1];
e_bh = zeros(N,7);
e_sc = zeros(N,7);
for i=1:N
% Now that you have X and E_bc
% TODO: Generate a random E_bh, you can use rotm2quat to convert
% the rotation matrix to a quaternion (be careful because it
% outputs in the format [qw, qx, qy, qz]) and append the
% transformation to e_bh
% Now that you have X, E_bc and E_bh
Rt_ebh = randSE3();
q_ebh = rotm2quat(Rt_ebh(1:3,1:3));
temp1 = q_ebh(1);
for j = 1:3
    q_ebh(j) = q_ebh(j+1);
end
q_ebh(4) = temp1;
T_ebh = Rt_ebh(1:3,4)';
e_bh(i,:) = [T_ebh, q_ebh];

% TODO: Find E_sc and append the transformation to e_sc
% E_bc = E_bh * X * E_sc
E_sc = (Rt_ebh*X) \ E_bc;
T_esc = E_sc(1:3,4);
T_esc = T_esc';
q_esc = rotm2quat(E_sc(1:3,1:3));
temp2 = q_esc(1);
for j = 1:3
    q_esc(j) = q_esc(j+1);
end
q_esc(4) = temp2;
e_sc(i,:) = [T_esc, q_esc];

end

end

% Generate a random SE3 transformation
function Rt = randSE3()
% TODO: Generate a random rotation matrix
q = randn(1, 4); % to generate a random quarternion
R = quat2rotm(q); % to convert the random quaternion into a ratation matrix
% TODO: Generate a random translation
T = randn(3, 1);
Rt = [ R T; 0 0 0 1];
end