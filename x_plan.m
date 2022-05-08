function [x] = x_plan(x0, xf, t)
%General Description:
    %This function returns a matrix, with each column representing the
    %position vector of the plate in world frame at a particular time t_i

%Parameters:
    %t: linspace representing a timeframe
    %n: number of position vectors recorded in T seconds
    %x0: initial position of tool [x, y, theta] (3x1 vector)
    %xf: final position of tool [x, y, theta] (3x1 vector)

n = length(t);
T = t(end)-t(1);
x = zeros(3,n);

for i=1:n
    x(:,i) = x0 + ((xf-x0)./T).*t(i);
    x(3, i) = deg2rad(x(3,i));
end