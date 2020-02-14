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

//用来更新位置和速度的类，分三种情况！！！
#ifndef ____PBA_EXPLICITDYNAMICS_H____
#define ____PBA_EXPLICITDYNAMICS_H____


#include "DynamicalState.h"
#include "Force.h"
#include "Constraint.h"
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

//=======================================================================
	//   Solvers that involve collisions

	class AdvancePositionWithCollisions : public GISolverBase
	{
	public:

		AdvancePositionWithCollisions(DynamicalState& pq, CollisionHandler& coll);
		~AdvancePositionWithCollisions() {}


		void init();
		void solve(const double dt);

	private:

		DynamicalState PQ;//属于AdvancePositionWithCollisions这个类，注意没有ExplicitDynamics类。
		CollisionHandler& CS;

	};

	GISolver CreateAdvancePosition(DynamicalState& pq, CollisionHandler& cs);

	//=================================================================================
	//   Solvers that involve constraints
	//V with CONS
	class AdvanceVelocityWithConstraint : public GISolverBase
	{
	public:

		AdvanceVelocityWithConstraint(DynamicalState& pq, Force& f, MultiConstraint& c);
		~AdvanceVelocityWithConstraint() {}


		void init();
		void solve(const double dt);

	private:

		DynamicalState PQ;
		Force force;
		MultiConstraint C;

	};


	GISolver CreateAdvanceVelocity( DynamicalState& pq, Force& f, MultiConstraint& c );


	//=============================================P WITH CONS
	class AdvancePositionWithConstraint : public GISolverBase 
	{
	public:
 
		AdvancePositionWithConstraint( DynamicalState& pq, MultiConstraint& c );
	~AdvancePositionWithConstraint(){}
 
 
		void init();
		void solve(const double dt);
 
	private:
 
		DynamicalState PQ;
		MultiConstraint C;
		double tolerance;
		int maxloops;
 
	};
 
	GISolver CreateAdvancePosition( DynamicalState& pq, MultiConstraint& cs );


	//===================================================================MY  test
	

	class AdvanceVelocityWithoutConstraint : public GISolverBase
	{
	public:

		AdvanceVelocityWithoutConstraint(DynamicalState& pq, Force& f, MultiConstraint& c);// { PQ = pq; C = c; };
		~AdvanceVelocityWithoutConstraint() {}


		void init() {}
		void solve(const double dt);

	private:

		DynamicalState PQ;
		MultiConstraint C;
		Force force;
		//double tolerance;
		//int maxloops;

	};

	GISolver CreateAdvanceVelocity2(DynamicalState& pq, Force& f, MultiConstraint& c);
}


#endif



