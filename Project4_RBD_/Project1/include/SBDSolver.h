#pragma once
//-------------------------------------------------------
//
//  SBDSolver.h
//
//  Solvers for Soft Body Dynamics
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#ifndef ____PBA_SBDSOLVER_H____
#define ____PBA_SBDSOLVER_H____


#include "SoftBodyState.h"
#include "Force.h"
#include "GISolver.h"
#include "CollisionHandler.h"



namespace pba
{


	class AdvanceSoftBodyPosition : public GISolverBase
	{
	public:

		AdvanceSoftBodyPosition(SoftBodyState& pq) {}
		~AdvanceSoftBodyPosition() {}


		void init() {}
		void solve(const double dt) {}

	private:

		SoftBodyState PQ;
		GISolver rbdsolver;

	};


	class AdvanceSoftBodyVelocity : public GISolverBase
	{
	public:

		AdvanceSoftBodyVelocity(SoftBodyState& pq, Force& f);
		~AdvanceSoftBodyVelocity() {}


		void init() {}
		void solve(const double dt);

	private:

		SoftBodyState PQ;
		Force force;
	};


	GISolver CreateAdvanceRotation(SoftBodyState& pq);
	GISolver CreateAdvanceAngularVelocity(SoftBodyState& pq, Force& f);


	class AdvanceSoftBodyPositionWithCollisions : public GISolverBase
	{
	public:

		AdvanceSoftBodyPositionWithCollisions(SoftBodyState& pq, ElasticSBDCollisionHandler& coll);
		~AdvanceSoftBodyPositionWithCollisions() {}


		void init() {}
		void solve(const double dt);

	private:

		SoftBodyState PQ;
		GISolver rbdsolver;
		ElasticSBDCollisionHandler& CS;

	};

	GISolver CreateAdvanceRotation(SoftBodyState& pq, ElasticSBDCollisionHandler& cs);




}


#endif
