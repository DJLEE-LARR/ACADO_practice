/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState x;
    DifferentialState y;
    DifferentialState z;
    DifferentialState phi;
    DifferentialState theta;
    DifferentialState psi;
    DifferentialState xdot;
    DifferentialState ydot;
    DifferentialState zdot;
    DifferentialState om_x;
    DifferentialState om_y;
    DifferentialState om_z;
    Control Ft;
    Control tau_x;
    Control tau_y;
    Control tau_z;
    Function acadodata_f2;
    acadodata_f2 << Ft;
    acadodata_f2 << tau_x;
    acadodata_f2 << tau_y;
    acadodata_f2 << tau_z;
    acadodata_f2 << x;
    acadodata_f2 << y;
    acadodata_f2 << z;
    acadodata_f2 << psi;
    DMatrix acadodata_M1;
    acadodata_M1.read( "multirotor_full_data_acadodata_M1.txt" );
    DVector acadodata_v1(8);
    acadodata_v1(0) = 9.810000E+00;
    acadodata_v1(1) = 0;
    acadodata_v1(2) = 0;
    acadodata_v1(3) = 0;
    acadodata_v1(4) = 1;
    acadodata_v1(5) = 2;
    acadodata_v1(6) = 3;
    acadodata_v1(7) = 0;
    DVector acadodata_v2(8);
    acadodata_v2(0) = 9.810000E+00;
    acadodata_v2(1) = 0;
    acadodata_v2(2) = 0;
    acadodata_v2(3) = 0;
    acadodata_v2(4) = 1;
    acadodata_v2(5) = 2;
    acadodata_v2(6) = 3;
    acadodata_v2(7) = 0;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x) == xdot;
    acadodata_f1 << dot(y) == ydot;
    acadodata_f1 << dot(z) == zdot;
    acadodata_f1 << dot(phi) == (cos(phi)*om_z*tan(theta)+om_x+om_y*sin(phi)*tan(theta));
    acadodata_f1 << dot(theta) == ((-sin(phi))*om_z+cos(phi)*om_y);
    acadodata_f1 << dot(psi) == (1/cos(theta)*om_y*sin(phi)+cos(phi)/cos(theta)*om_z);
    acadodata_f1 << dot(xdot) == (cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*Ft;
    acadodata_f1 << dot(ydot) == (cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi))*Ft;
    acadodata_f1 << dot(zdot) == (-9.81000000000000049738e+00+Ft*cos(phi)*cos(theta));
    acadodata_f1 << dot(om_x) == (1.00000000000000002082e-02*om_y*om_z-1.00000000000000002082e-02*om_z*om_y+tau_x)*1.00000000000000000000e+02;
    acadodata_f1 << dot(om_y) == (-1.00000000000000002082e-02*om_x*om_z+1.00000000000000002082e-02*om_z*om_x+tau_y)*1.00000000000000000000e+02;
    acadodata_f1 << dot(om_z) == (1.00000000000000002082e-02*om_x*om_y-1.00000000000000002082e-02*om_y*om_x+tau_z)*1.00000000000000000000e+02;

    OCP ocp1(0, 10, 200);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2, acadodata_v2);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, x == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, y == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, z == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, phi == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, theta == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, psi == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, xdot == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, ydot == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, zdot == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, om_x == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, om_y == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, om_z == 0.00000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= Ft <= 2.00000000000000000000e+01);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-05 );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 
