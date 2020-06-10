function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
if H(3, 3) < 0
    H = -H;
end

[nrows, ncols] = size(render_points);
proj_points=zeros(nrows, 2);
H1 = H(:,1);
H2 = H(:,2);
H3 = H(:,3);
A = [H1, H2, cross(H1, H2)];
[U, ~, V] = svd(A);
s = [1,1,det(U*V')];
R = U*diag(s)*V';
t = H3 / norm(H1);

% YOUR CODE HERE: Project the points using the pose
for i = 1:nrows    
    X = render_points(i,:)';    
    Xc = K * (R * X + t);    
    proj_points(i, :) = [Xc(1) / Xc(3), Xc(2) / Xc(3)];
end

end

