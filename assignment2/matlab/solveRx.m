function Rx=solveRx( alphas, betas )
% alphas: A 3xN matrix representing the skew symmetric matrices. That
%         is, alphas=[α1,...,αN]
% betas: A 3xN matrix representing the skew symmetric matrices. That
%        is, betas=[β1,...,βN]
% return: The least squares solution to the matrix Rx

[~, N] = size(alphas);          % to get the number of how many data points
M = zeros(3);

for i = 1:N
    M = M + betas(:, i) * alphas(:, i)'; 
end

Rx = (M' * M)^(-0.5) * M';

end
