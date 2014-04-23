function [desire_state] = getDesiredStateFromTrajectory(coefficient, time_stamp)
%GETDESIREDSTATEFROMTRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
desire_state = zeros(1, 16);
coefficient = coefficient';
%t = [time_stamp .^ 5 time_stamp .^ 4 time_stamp .^ 3 time_stamp .^ 2 time_stamp 1];
t = [1; time_stamp; time_stamp .^ 2; time_stamp .^ 3; time_stamp .^ 4; time_stamp .^ 5]';
% x y z
desire_state(1) = sum(coefficient(1, :) .* t);
desire_state(2) = sum(coefficient(2, :) .* t);
desire_state(3) = sum(coefficient(3, :) .* t);
% d_x d_y d_z
new_coef = polyder(coefficient(1, :));
desire_state(4) = sum(new_coef .* t(end - size(new_coef, 2) + 1 : end));
new_coef = polyder(coefficient(2, :));
desire_state(5) = sum(new_coef .* t(end - size(new_coef, 2) + 1 : end));
new_coef = polyder(coefficient(3, :));
desire_state(6) = sum(new_coef .* t(end - size(new_coef, 2) + 1 : end));
% dd_x dd_y dd_z
new_coef = polyder(polyder(coefficient(1, :)));
desire_state(7) = sum(new_coef .* t(end - size(new_coef, 2) + 1 : end));
new_coef = polyder(polyder(coefficient(2, :)));
desire_state(8) = sum(new_coef .* t(end - size(new_coef, 2) + 1 : end));
new_coef = polyder(polyder(coefficient(3, :)));
desire_state(9) = sum(new_coef .* t(end - size(new_coef, 2) + 1 : end));
% roll pitch yaw
desire_state(10) = 0;
desire_state(11) = 0;
desire_state(12) = sum(coefficient(4, :) .* t);
% p q r
desire_state(13) = 0;
desire_state(14) = 0;
desire_state(15) = 0;

end

