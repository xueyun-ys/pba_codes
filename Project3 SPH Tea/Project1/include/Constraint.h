#pragma once
//-------------------------------------------------------
//
//  Constraint.h
//
//  Base class and refcounted pointer for constraints
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_CONSTRAINT_H____
#define ____PBA_CONSTRAINT_H____

#include "Vector.h"
#include "Matrix.h"
#include "DynamicalState.h"
//#include "RigidBodyState.h"
//#include "SoftBodyState.h"
#include "LinearAlgebra.h"
#include <iostream>


namespace pba
{


	class ConstraintBase
	{
	public:

		ConstraintBase() : Ks(0.0), Kf(0.0) {};

		virtual double compute(DynamicalState& s) { std::cout << "calling ConstraintBase::compute(DynamicalState) base class virtual method\n"; return 0; }
		//virtual double compute(RigidBodyState& s) { std::cout << "calling ConstraintBase::compute(RigidBodyState) base class virtual method\n"; return 0; }
		//virtual double compute(SoftBodyState& s) { std::cout << "calling ConstraintBase::compute(SoftBodyState) base class virtual method\n"; return 0; }

		virtual Vector grad(DynamicalState& s, int index) { std::cout << "calling ConstraintBase::grad(DynamicalState, int) base class virtual method\n"; return Vector(0, 0, 0); }
		//virtual Vector grad(RigidBodyState& s, int index) { std::cout << "calling ConstraintBase::grad(RigidBodyState, int) base class virtual method\n"; return Vector(0, 0, 0); }
		//virtual Vector grad(SoftBodyState& s, int index) { std::cout << "calling ConstraintBase::grad(SoftBodyState, int) base class virtual method\n"; return Vector(0, 0, 0); }

		virtual Matrix gradgrad(DynamicalState& s, int i, int j) { std::cout << "calling ConstraintBase::gradgrad(DynamicalState, double) base class virtual method\n"; return unitMatrix()*0.0; }
		//virtual Matrix gradgrad(RigidBodyState& s, int i, int j) { std::cout << "calling ConstraintBase::gradgrad(RigidBodyState, double) base class virtual method\n"; return unitMatrix()*0.0; }
		//virtual Matrix gradgrad(SoftBodyState& s, int i, int j) { std::cout << "calling ConstraintBase::gradgrad(SoftBodyState, double) base class virtual method\n"; return unitMatrix()*0.0; }

		virtual void solve(DynamicalState& s, double tol, int maxloop) { std::cout << "calling ConstraintBase::solve(DynamicalState,double,int) base class virtual method\n"; }
		//virtual void solve(RigidBodyState& s, double tol, int maxloop) { std::cout << "calling ConstraintBase::solve(RigidBodyState,double,int) base class virtual method\n"; }
		//virtual void solve(SoftBodyState& s, double tol, int maxloop) { std::cout << "calling ConstraintBase::solve(SoftBodyState,double,int) base class virtual method\n"; }


		virtual ~ConstraintBase() {};


		double get_Ks() const { return Ks; };
		double get_Kf() const { return Kf; };

		void set_Ks(double v) { Ks = v; };
		void set_Kf(double v) { Kf = v; };


	private:

		double Ks, Kf;



	};

	typedef std::shared_ptr<ConstraintBase> Constraint;


	class MultiConstraintBase
	{
	public:

		MultiConstraintBase() {};

		double compute(DynamicalState& s, size_t alpha) { return constraints[alpha]->compute(s); };
		//double compute(RigidBodyState& s, size_t alpha) { return constraints[alpha]->compute(s); };
		//double compute(SoftBodyState& s, size_t alpha) { return constraints[alpha]->compute(s); };

		Vector grad(DynamicalState& s, size_t alpha, int index) { return constraints[alpha]->grad(s, index); }
		//Vector grad(RigidBodyState& s, size_t alpha, int index) { return constraints[alpha]->grad(s, index); }
		//Vector grad(SoftBodyState& s, size_t alpha, int index) { return constraints[alpha]->grad(s, index); }

		Matrix gradgrad(DynamicalState& s, size_t alpha, int i, int j) { return constraints[alpha]->gradgrad(s, i, j); }//用multiconstrain把各个类型的constrain存储起来,以便分别调用;这里调用了
		//包包里的一个constraint然后得到了返回值，相当于包装了一层
		//Matrix gradgrad(RigidBodyState& s, size_t alpha, int i, int j) { return constraints[alpha]->gradgrad(s, i, j); }
		//Matrix gradgrad(SoftBodyState& s, size_t alpha, int i, int j) { return constraints[alpha]->gradgrad(s, i, j); }


		void solve(DynamicalState& s, double tol, int maxloop);
		//void solve(RigidBodyState& s, double tol, int maxloop) {};
		//void solve(SoftBodyState& s, double tol, int maxloop) {};



		~MultiConstraintBase() {};

		void addConstraint(Constraint& c) { constraints.push_back(c); }

		const Constraint get_constraint(size_t c) const { return constraints[c]; };
		Constraint get_constraint(size_t c) { return constraints[c]; };

		size_t nb() const { return constraints.size(); };


	private:

		std::vector<Constraint> constraints;

	};

	typedef std::shared_ptr<MultiConstraintBase> MultiConstraint;

	MultiConstraint CreateMultiConstraint();



	

}
#endif