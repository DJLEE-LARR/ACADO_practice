clc; clear;

%% rotation matrix
syms phi theta psi

Rx = [1 0 0;
    0 cos(phi) -sin(phi);
    0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];


Re3 = Rz*Ry*Rx*[0;0;1];

simplify(Re3)

%% coriolis term

syms om_x om_y om_z I_x I_y I_z

Om = [om_x;om_y;om_z];
I = diag([I_x I_y I_z]);

simplify(-cross(Om,I*Om))

%% euler angle jacobian to angular velocity

% PHIdot = Q*[om_x;om_y;om_z]

syms phi theta 

Q = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
     0 cos(phi) -sin(phi);
     0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
