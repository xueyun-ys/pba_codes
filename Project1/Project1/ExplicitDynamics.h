#pragma once
//-------------------------------------------------------
//
//  ExplicitDynamics.h
//
//  Various Dynamics Models Using Explicit Solvers
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#ifndef ____PBA_EXPLICITDYNAMICS_H____
#define ____PBA_EXPLICITDYNAMICS_H____


#include "DynamicalState.h"
#include "Force.h"
//#include "Constraint.h"
#include "GISolver.h"
#include "CollisionHandler.h"



namespace pba
{


	//   Solvers with no collisions or constraints

	class AdvancePosition : public GISolverBase
	{
	public:

		AdvancePosition(DynamicalState& pq);
		~AdvancePosition() {}


		void init();
		void solve(const double dt);

	private:

		DynamicalState PQ;

	};


	class AdvanceVelocity : public GISolverBase
	{
	public:

		AdvanceVelocity(DynamicalState& pq, Force& f);
		~AdvanceVelocity() {}


		void init();
		void solve(const double dt);

	private:

		DynamicalState PQ;
		Force force;

	};


	GISolver CreateAdvancePosition(DynamicalState& pq);
	GISolver CreateAdvanceVelocity(DynamicalState& pq, Force& f);


	//   Solvers that involve collisions

	class AdvancePositionWithCollisions : public GISolverBase
	{
	public:

		AdvancePositionWithCollisions(DynamicalState& pq, CollisionHandler& coll);
		~AdvancePositionWithCollisions() {}


		void init();
		void solve(const double dt);

	private:

		DynamicalState PQ;//属于AdvancePositionWithCollisions这个类，注意没有ExplicitDynamics。
		CollisionHandler& CS;

	};

	GISolver CreateAdvancePosition(DynamicalState& pq, CollisionHandler& cs);

	//   Solvers that involve constraints

	//class AdvanceVelocityWithConstraint : public GISolverBase
	//{
	//public:

	//	AdvanceVelocityWithConstraint(DynamicalState& pq, Force& f);// , Constraint& c);
	//	~AdvanceVelocityWithConstraint() {}


	//	void init();
	//	void solve(const double dt);

	//private:

	//	DynamicalState PQ;
	//	Force force;
	//	//Constraint C;

	//};


	GISolver CreateAdvanceVelocity(DynamicalState& pq, Force& f);// , Constraint& c);


	class AdvancePositionWithConstraint : public GISolverBase
	{
	public:

		AdvancePositionWithConstraint(DynamicalState& pq);//, Constraint& c);
		~AdvancePositionWithConstraint() {}


		void init();
		void solve(const double dt);

	private:

		DynamicalState PQ;
		//Constraint C;
		double tolerance;
		int maxloops;

	};

	GISolver CreateAdvancePosition(DynamicalState& pq);// , Constraint& cs);


}


#endif
