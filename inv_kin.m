function [q] = inv_kin(x_mat, t, params)
%General Description:
    %This function returns a matrix, with each column representing the
    %joint values corresponding to a specified position at some time ti - 
    %Note that there are 1 to 2 solutions to each joint value

%Parameters:
    %x_mat: matrix of n position vectors of the robot over timeframe t
    %t: linspace representing a timeframe
    %params: [r, L, R]



r = params(1);
L = params(2);
R = params(3);

n = length(t);
q = zeros(6, n);


for i=1:n

    syms theta1 theta2 theta3 phi;

    x = x_mat(1,i);
    y = x_mat(2,i);
    phi = deg2rad(x_mat(3,i));

    A=[x,y];
    B=[x+r*cos(phi),y+r*sin(phi)];
    C=[x+r*cos(phi+pi/3),y+r*sin(phi+pi/3)];
    
    D=[R*cos(theta2), R*sin(theta2)];
    E=[R*cos(theta3), R*sin(theta3)];
    F=[R*cos(theta1), R*sin(theta1)];
    
    r1=A-D;
    r2=B-E;
    r3=C-F;
    
    eqn1=(norm(r1)^2-L^2==0);
    theta2=vpa(solve(eqn1,theta2),4);
    eqn2=(norm(r2)^2-L^2==0);
    theta3=vpa(solve(eqn2,theta3),4);
    eqn3=(norm(r3)^2-L^2==0);
    theta1=vpa(solve(eqn3,theta1),4);

    q(1:2,i) = theta1;
    q(3:4,i) = theta2;
    q(5:6,i) = theta3;


    
end