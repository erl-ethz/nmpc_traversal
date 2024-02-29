/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
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


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <nlohmann/json.hpp>
#include <time.h>

int main( ){

	USING_NAMESPACE_ACADO

	// INTRODUCE THE VARIABLES:
	// -------------------------
  DifferentialState     p_x, p_y, p_z;					// position
  DifferentialState     v_x, v_y, v_z;					// velocity
  DifferentialState     q_w, q_x, q_y, q_z;			// orientation (quaternion)
	DifferentialState			w_x, w_y, w_z;					// angular velocity (body)

	OnlineData f_e_x, f_e_y, f_e_z;				// external force
	OnlineData t_e_x, t_e_y, t_e_z;				// external torque
	OnlineData v_ref_x, v_ref_y, v_ref_z;										// needed for the desired impedance dynamics

  // Control v_desired_x, v_desired_y, v_desired_z;
	// Control a_desired_x, a_desired_y, a_desired_z;

	// Control w_x, w_y, w_z;
	Control T;
	Control tau_x, tau_y, tau_z;

	IntermediateState roll, pitch, yaw;
	IntermediateState f_e_x_world, f_e_y_world, f_e_z_world;

	// load impedance parameters
	std::ifstream strGeneralSetup("configuration/general_setup.json");
	nlohmann::json jsonGeneralSetup;
	jsonGeneralSetup << strGeneralSetup;
	const double M   = jsonGeneralSetup["M"];
	const double I_x = jsonGeneralSetup["I_x"];
	const double I_y = jsonGeneralSetup["I_y"];
	const double I_z = jsonGeneralSetup["I_z"];
	const double D_x = jsonGeneralSetup["D_x"];
	const double D_y = jsonGeneralSetup["D_y"];
	const double D_z = jsonGeneralSetup["D_z"];
	const double g = 9.81;


	roll = atan(2 * (q_w * q_x + q_y * q_z)/( 1 - 2 * (q_x * q_x + q_y * q_y)) );
	pitch = asin(2 * (q_w * q_y - q_z * q_x));
	yaw = atan(2 * (q_y * q_x + q_w * q_z)/( 1 - 2 * (q_z * q_z + q_y * q_y)) );

	f_e_x_world = f_e_x*(1-2*q_y*q_y-2*q_z*q_z) + f_e_y*(2*(q_x*q_y-q_w*q_z)) + f_e_z*(2*(q_w*q_y+q_x*q_z));
	f_e_y_world = f_e_x*(2*(q_x*q_y+q_w*q_z)) + f_e_y*(1-2*q_x*q_x-2*q_z*q_z) + f_e_z*(2*(q_y*q_z-q_w*q_x));
	f_e_z_world = f_e_x*(2*(q_x*q_z-q_w*q_y)) + f_e_y*(2*(q_y*q_z+q_w*q_x)) + f_e_z*(1-2*q_x*q_x-2*q_y*q_y);

	// DEFINE A DIFFERENTIAL EQUATION:
	// -------------------------------
	DifferentialEquation f;

	IntermediateState a_x = 2 * ( q_w * q_y + q_x * q_z ) * T + f_e_x_world/M;
	IntermediateState a_y = 2 * ( q_y * q_z - q_w * q_x ) * T + f_e_y_world/M;
	IntermediateState a_z = ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T - g + f_e_z_world/M;
	IntermediateState a_impedance_x = - D_x/M*(v_x - v_ref_x);
	IntermediateState a_impedance_y = - D_y/M*(v_y - v_ref_y) + f_e_y_world/M;
	IntermediateState a_impedance_z = - D_z/M*(v_z - v_ref_z);


  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(v_x) ==  a_x;
  f << dot(v_y) ==  a_y;
  f << dot(v_z) ==  a_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
	f << dot(w_x) == ((I_y - I_z) * w_y * w_z + t_e_x + tau_x)/I_x;
	f << dot(w_y) == ((I_z - I_x) * w_x * w_z + t_e_y + tau_y)/I_y;
	f << dot(w_z) == ((I_x - I_y) * w_x * w_y + t_e_z + tau_z)/I_z;


	// DEFINE WEIGHT OF OBJECTIVE FUNCTION:
	// -----------------------------

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 1.0;       // Time horizon [s]
  const double dt = 0.05;          // Discretization time [s] 
  const int N = round(t_end/dt);  // Number of nodes

	OCP ocp( t_start, t_end, N);

	Function h, hN;


	h << p_y << p_z
		<< v_x << v_y << v_z 
		<< yaw 
		<< w_x << w_y << w_z
		<< tau_x << tau_y << tau_z << T
		<< a_impedance_x - a_x << a_impedance_y - a_y << a_impedance_z - a_z;
	hN << p_y << p_z << v_z;

	// For code generation, references are set during run time
	BMatrix Q_sparse(h.getDim(), h.getDim());
	BMatrix QN_sparse(hN.getDim(), hN.getDim());
	Q_sparse.setIdentity();
	QN_sparse.setIdentity();

	ocp.minimizeLSQ( Q_sparse, h);
	ocp.minimizeLSQEndTerm( QN_sparse, hN );

	// Progression term

	ocp.subjectTo( f );

	// Position Impedance dynamics constraint
	// ocp.subjectTo(a_x - a_impedance_x == 0);
	// ocp.subjectTo(a_y - a_impedance_y == 0);
	// ocp.subjectTo(a_z - a_impedance_z == 0);

	const double thrust_max = 20;		// m/s^2
	const double thrust_min = 0.0;
	const double max_inclination = 20; 		// degree
	// const double max_angular_vel_xy = 3;
	// const double max_angular_vel_yaw = 1;
	const double max_tau_xy = 3;
	const double max_tau_z = 1;
	
	// Euler angles
	ocp.subjectTo( -max_inclination <= roll*180/M_PI <= max_inclination);
	ocp.subjectTo( -max_inclination <= pitch*180/M_PI <= max_inclination);

	// Thrust
	ocp.subjectTo( thrust_min <= T <= thrust_max);

	// floor is lava
	ocp.subjectTo( 0 <= p_z <= 1000);

	// Omega
	ocp.subjectTo( -max_tau_xy <= tau_x <= max_tau_xy);
	ocp.subjectTo( -max_tau_xy <= tau_y <= max_tau_xy);
	ocp.subjectTo( -max_tau_z <= tau_z <= max_tau_z);

	ocp.setNOD(9);			// Set the number of OnlineData

	// DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	// ---------------------------------------------------
	// For code generation, we can set some properties.
	// The main reason for a setting is given as comment.
	OCPexport mpc(ocp);

	mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
	mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
	mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
	mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
	mpc.set(NUM_INTEGRATOR_STEPS,   N);
	mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
	mpc.set(HOTSTART_QP,            YES);
	mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
	mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
	mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
	mpc.set(USE_SINGLE_PRECISION,        YES);           // Single precision

	// Do not generate tests, makes or matlab-related interfaces.
	mpc.set(GENERATE_TEST_FILE,          YES);
	mpc.set(GENERATE_MAKE_FILE,          YES);
	mpc.set(GENERATE_MATLAB_INTERFACE,   NO);
	mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

	// Finally, export everything.
	if(mpc.exportCode("mpc_impedance_based_codegen") != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}



