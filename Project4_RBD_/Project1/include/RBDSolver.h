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

	class AdvanceAngularVelocityAndVelocity : public GISolverBase//ÿһ������һ��solver
	{
	public:

		AdvanceAngularVelocityAndVelocity(RigidBodyState& pq, Force& f);
		~AdvanceAngularVelocityAndVelocity() {}


		void init();
		void solve(const double dt);

	private:

		RigidBodyState PQ;
		Torque tau;//�������Ǹ��൫���õ�ʱ���ڳ�ʼ��������ʱ��������������Ķ���

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
