clear;

BEGIN_ACADO;

    acadoSet('problemname','double_integrator');
    
    DifferentialState x xdot;
%     DifferentialState COST;
    Control F;
    
    %% differential equation
    
    f = acado.DifferentialEquation();
    f.add(dot(x) == xdot);
    f.add(dot(xdot) == F);
    
%     f.add(dot(COST) == 1/2*(50*(x-2.0)^2 + 50*(xdot)^2) );
    
    ocp = acado.OCP(0.0, 3, 50);
    algo = acado.OptimizationAlgorithm(ocp);
    
    S = [1 0 0; 0 10 0;0 0 1];
    h = {F, x, xdot};
    r = [0.0, 1.0, 0.0];
    
    Se = [50 0; 0 50];
    he = {x,xdot};
    re = [1.0, 0.0];
    
%     ocp.minimizeMayerTerm(COST);
    ocp.minimizeLSQ(S,h,r);
    ocp.minimizeLSQEndTerm(Se,he,re);
    
    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', x == 0.0 );
    ocp.subjectTo( 'AT_START', xdot == 0.0 );
%     ocp.subjectTo( 'AT_START', COST == 0.0 );
    
    ocp.subjectTo (-10 <= F <= 10);
    
%     algo.set('KKT_TOLERANCE', 1e-4 );
    
END_ACADO;

out = double_integrator_RUN();

draw;
    