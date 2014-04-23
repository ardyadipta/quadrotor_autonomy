% cubic Cubic polynomial trajectory with specified end p & v, and tf.
%
% q = cubic(q0,q1,qdot0,qdot1,tf) generates a cubic polynomial
% trajectory comprised of 100 equally-spaced points in time.
%
% q0, q1 = initial and final joint position.
% qdot0, qdot1 = initial and final joint velocity.
% tf = total movement time.
% q = vector of joint positions, equally spaced in time.
function q = cubic(q0,q1,qdot0,qdot1,tf)
a = zeros(4,1);
% Generate coefficients of cubic polynomial.
a(1) = q0;
a(2) = qdot0;
a(3) = (3 * (q1-q0) - tf * (2 * qdot0 + qdot1)) / tfˆ2;
a(4) = (-2 * (q1-q0) + tf * (qdot0 + qdot1)) / tfˆ3;
% Divide time into 100 intervals.
dt = tf/100;
% This Matlab trick creates an array from 0 to tf with increment dt.
t = 0:dt:tf;
% Generate joint trajectory.
q = a(1) + t .* (a(2) + t .* (a(3) + t * a(4)));
% Plot the results. Control axis spacing.
plot(q);
axis([0 100 -20 10]);
% Save the plot for printing on a postscript printer.
print -deps2 cubic.eps