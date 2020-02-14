#pragma once
//*******************************************************************
//
//   RK4.h
//
// Runge Kutta 4th order solver
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//*******************************************************************

#ifndef __PBA_RK4_H__
#define __PBA_RK4_H__

#include "GISolver.h"
#include "DynamicalState.h"
#include "Force.h"


namespace pba {

	class RungeKuttaFourthOrderSolver : public GISolverBase
	{
	public:

		RungeKuttaFourthOrderSolver(DynamicalState& pq, Force& f) :
			PQ(pq),
			force(f)
		{}

		~RungeKuttaFourthOrderSolver() {};

		void init();
		void solve(const double dt);


	private:

		DynamicalState PQ;
		Force force;

	};



	GISolver CreateRK4Solver(DynamicalState& pq, Force& f);

}


#endif