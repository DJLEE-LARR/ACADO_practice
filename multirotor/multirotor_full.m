clc; clear; close all;
   
BEGIN_ACADO;

    acadoSet('problemname','multirotor_full');

    DifferentialState x y z phi theta psi;
    DifferentialState xdot ydot zdot om_x om_y om_z;

    Control Ft;
    Control tau_x;
    Control tau_y;
    Control tau_z;

    Re3 = [sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
       cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
       cos(phi)*cos(theta)];
    Q = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
        0 cos(phi) -sin(phi);
        0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    

    %% parameters

    m_ = 1.0;
    I_ = diag([0.01 0.01 0.01]);
    g_ = 9.81;

    coriolis = cross([om_x;om_y;om_z],I_*[om_x;om_y;om_z]);
    
    %% Differential Equation
    f = acado.DifferentialEquation();

    f.add(dot(x) == xdot);
    f.add(dot(y) == ydot);
    f.add(dot(z) == zdot);
    
    f.add(dot(phi) == Q(1,:)*[om_x;om_y;om_z]);
    f.add(dot(theta) == Q(2,:)*[om_x;om_y;om_z]);
    f.add(dot(psi) == Q(3,:)*[om_x;om_y;om_z]);
    
    f.add(dot(xdot) == (Ft/m_*Re3(1,1)));
    f.add(dot(ydot) == (Ft/m_*Re3(2,1)));
    f.add(dot(zdot) == (Ft/m_*Re3(3,1) - g_));
%     f.add(dot(zdot) == (Ft/m_*Re3(3,1)));

    f.add(dot(om_x) == 1/I_(1,1)*(-coriolis(1,1) + tau_x));
    f.add(dot(om_y) == 1/I_(2,2)*(-coriolis(2,1) + tau_y));
    f.add(dot(om_z) == 1/I_(3,3)*(-coriolis(3,1) + tau_z));
    
    %% Optimal Control Problem
    t0 = 0.0; tf = 10.0; dt = 0.05;
    ocp = acado.OCP(0.0, 10.0, (tf-t0)/dt);

    S = diag([1 1 1 1 ...
              1 1 1 1]);
    h = {Ft, tau_x, tau_y, tau_z, ...
         x, y, z, psi};
    r = [m_*g_, 0, 0, 0, ...
        1.0, 2.0, 3.0, 0.0];

%     Se = diag([20 20 40 10]);
%     he = {x, y, z, psi};
%     re = [1.0, 2.0, 3.0, 0.0];

    ocp.minimizeLSQ(S,h,r);
%     ocp.minimizeLSQEndTerm(Se,he,re);

    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', x == 0.0);
    ocp.subjectTo( 'AT_START', y == 0.0);
    ocp.subjectTo( 'AT_START', z == 0.0);
    ocp.subjectTo( 'AT_START', phi == 0.0);
    ocp.subjectTo( 'AT_START', theta == 0.0);
    ocp.subjectTo( 'AT_START', psi == 0.0);
    ocp.subjectTo( 'AT_START', xdot == 0.0);
    ocp.subjectTo( 'AT_START', ydot == 0.0);
    ocp.subjectTo( 'AT_START', zdot == 0.0);
    ocp.subjectTo( 'AT_START', om_x == 0.0);
    ocp.subjectTo( 'AT_START', om_y == 0.0);
    ocp.subjectTo( 'AT_START', om_z == 0.0);
    
    ocp.subjectTo( 0.0 <= Ft <= 20.0 );
%     ocp.subjectTo( -5.0 <= phidot_d <= 5.0);
%     ocp.subjectTo( -5.0 <= thetadot_d <= 5.0);
%     ocp.subjectTo( -5.0 <= psidot_d <= 5.0);

    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp);
    algo.set( 'KKT_TOLERANCE', 1e-5 );
%     algo.initializeControls([0 m_*g_ 0 0 0]);
    
END_ACADO;

% RUN THE TEST
out = multirotor_full_RUN();

draw;
