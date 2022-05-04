function tx=solveTx( RA, tA, RB, tB, RX )
% RA: a 3x3xN matrix with all the rotations matrices R_(A_i )
% tA: a 3xN matrix with all the translation vectors t_(A_i )
% RB: a 3x3xN matrix with all the rotations matrices R_(B_i )
% tB: a 3xN matrix with all the translation vectors t_(B_i )
% RX: the 3x3 rotation matrix Rx
% return: the 3x1 translation vector tx

[~, ~, N] = size(RA);               % the num of data points

% left_matrix * Tx = right_matrix
left_matrix = zeros(3*N, 3);
right_matrix = zeros(3*N, 1);
for i = 1:N
    left_matrix(3*i-2:3*i, :) = eye(3) - RA(:, :, i);
    right_matrix(3*i-2:3*i, :) = tA(:, i) - RX * tB(:, i);
end

tx = left_matrix \ right_matrix;
end
