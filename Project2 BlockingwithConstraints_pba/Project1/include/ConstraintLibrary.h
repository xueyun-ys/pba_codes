#pragma once
//-------------------------------------------------------
//
//  ConstraintLibrary.h
//
//  A collection of constraint implementations
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_CONSTRAINTLIBRARY_H____
#define ____PBA_CONSTRAINTLIBRARY_H____

#include "Constraint.h"
#include "DynamicalState.h"
//#include "RigidBodyState.h"
//#include "SoftBodyState.h"

namespace pba
{


	//�洢��constrainbase��constrain�ĸ��������࣬�̳й�ϵ
	class ParticleOnSphereConstraint : public ConstraintBase
	{
	public:

		ParticleOnSphereConstraint(double radius, Vector& center, int particle_id) :
			R(radius),
			P(center),
			id(particle_id)
		{};

		~ParticleOnSphereConstraint() {};

		double compute(DynamicalState& s);
		Vector grad(DynamicalState& s, int index);
		Matrix gradgrad(DynamicalState& s, int i, int j);

		void solve(DynamicalState& s, double tol, int maxloop);

		void set_id(int d);

		double get_radius() { return R; }

		Vector get_Spherecenter() { return P; }

	private:

		double R;//�뾶
		Vector P;//�������
		int id;//particle_id

	};

	Constraint CreateParticleOnSphereConstraint(double R, Vector cen, int p);

}
#endif
