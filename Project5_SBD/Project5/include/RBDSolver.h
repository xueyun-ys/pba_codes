#pragma once
//-------------------------------------------------------
//
//  RBDSolver.h
//
//  Solvers for Rigid Body Dynamics
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#ifndef ____PBA_RBDSOLVER_H____
#define ____PBA_RBDSOLVER_H____


#include "RigidBodyState.h"
#include "Torque.h"
#include "GISolver.h"
#include "CollisionHandler.h"



namespace pba
{


	class AdvanceRotationAndCOM : public GISolverBase
	{
	public:

		AdvanceRotationAndCOM(RigidBodyState& pq);
		~AdvanceRotationAndCOM() {}


		void init();
		void solve(const double dt);

	private:

		RigidBodyState PQ;

	};
	//=============================================================

	class AdvanceAngularVelocityAndVelocity : public GISolverBase//每一个都是一个solver
	{
	public:

		AdvanceAngularVelocityAndVelocity(RigidBodyState& pq, Force& f);
		~AdvanceAngularVelocityAndVelocity() {}


		void init();
		void solve(const double dt);

	private:

		RigidBodyState PQ;
		Torque tau;//在这里是父类但是用的时候在初始化这个类的时候把它变成了子类的对象

	};


	GISolver CreateAdvanceRotation(RigidBodyState& pq);
	GISolver CreateAdvanceAngularVelocity(RigidBodyState& pq, Force& f);

	//================================================================
	class AdvanceRotationWithCollisions : public GISolverBase
	{
	public:

		AdvanceRotationWithCollisions(RigidBodyState& pq, ElasticRBDCollisionHandler& coll);
		~AdvanceRotationWithCollisions() {}


		void init();
		void solve(const double dt);

	private:

		RigidBodyState PQ;
		ElasticRBDCollisionHandler& CS;

	};

	GISolver CreateAdvanceRotation(RigidBodyState& pq, ElasticRBDCollisionHandler& cs);




}


#endif
