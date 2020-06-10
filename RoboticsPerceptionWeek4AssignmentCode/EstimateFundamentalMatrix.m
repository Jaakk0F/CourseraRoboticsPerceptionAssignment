function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2
%%% step 1: solving least square problem
A = [x1(1)*x2(1), x1(1)*x2(2), x1(1), x1(2)*x2(1), x1(2)*x2(2), x1(2), x2(1), x2(2), 1];
[~, ~, V] = svd(A);
X = V(:, end);
F = reshape(X, 3, 3);

%%% step 2: normalize F
[U, S, V] = svd(F);
S(3, 3) = 0;
F = U * S * V';
F = F / norm(F);