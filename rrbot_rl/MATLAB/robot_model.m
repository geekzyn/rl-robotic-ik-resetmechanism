clc; clear all; close all;

%%%Boolean Parameters
plot = 1;

syms l1 l2 l3 theta_1 theta_2 Px Py Pz;
syms t1 t2 t3;
syms xEE yEE zEE;

%%% Transformation Matrix
t1 = [cos(0), -sin(0)*cos(pi/2), sin(0)*sin(pi/2), 0.1*cos(0);
    sin(0), cos(0)*cos(pi/2), -cos(0)*sin(pi/2), 0.1*sin(0);
    0, sin(pi/2), cos(pi/2), 2;
    0, 0, 0, 1];
                           

t2 = [cos(theta_1), -sin(theta_1)*cos(0), sin(theta_1)*sin(0), 1*cos(theta_1);
    sin(theta_1), cos(theta_1)*cos(0), -cos(theta_1)*sin(0), 1*sin(theta_1);
    0, sin(0), cos(0), 0.1;
    0, 0, 0, 1];


t3 = [cos(theta_2), -sin(theta_2)*cos(0), sin(theta_2)*sin(0), 1*cos(theta_2);
  sin(theta_2), cos(theta_2)*cos(0), -cos(theta_2)*sin(0), 1*sin(theta_2);
  0, sin(0), cos(0), 0.1;
  0, 0, 0, 1];

T = t1*t2*t3;

%%% Forward Kinematics
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);

Px_matlab = matlabFunction(Px, 'Vars', [theta_1, theta_2]);
Py_matlab = matlabFunction(Py, 'Vars', [theta_1, theta_2]);
Pz_matlab = matlabFunction(Pz, 'Vars', [theta_1, theta_2]);


theta = deg2rad([linspace(-360, 360, 100);linspace(-360, 360, 100)]);
x = Px_matlab(theta(1, :), theta(2, :));
y = Py_matlab(theta(1, :), theta(2, :));
z = Pz_matlab(theta(1, :), theta(2, :));


if plot
    figure(1);
    plot3(x, y, z,'-*','MarkerIndices',[1, 50, 100]);
    grid on;
end

%%% Inverse Kinematics
xEq = xEE == Px;
yEq = yEE == Py;
zEq = zEE == Pz;

S = solve([xEq yEq], [theta_1 theta_2]);

th11_matlab = matlabFunction(S.theta_1(1), 'Vars', [xEE yEE]);
th12_matlab = matlabFunction(S.theta_1(2), 'Vars', [xEE yEE]);
th21_matlab = matlabFunction(S.theta_2(1), 'Vars', [xEE yEE]);
th22_matlab = matlabFunction(S.theta_2(2), 'Vars', [xEE yEE]);

%%% Jacobian
J = jacobian([Px Py Pz], [theta_1, theta_2]);