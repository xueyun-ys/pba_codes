#pragma once
//*******************************************************************
//
//   GISolver.h
//
// Base class for Geometric Integration solvers
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//*******************************************************************

#ifndef __PBA_GISOLVER_H__
#define __PBA_GISOLVER_H__

#include <cmath>
#include <memory>



namespace pba {

	class GISolverBase
	{
	public:

		GISolverBase() {}

		virtual void init() = 0;
		virtual void solve(const double dt) = 0;
		virtual ~GISolverBase() {};


	protected:

	};


	typedef std::shared_ptr<GISolverBase> GISolver;



	class GISolverSubstep : public GISolverBase
	{
	public:
		GISolverSubstep(GISolver& s, int nbsteps) :
			_solver(s), _steps(nbsteps)
		{}

		~GISolverSubstep() {}

		void init() { _solver->init(); }

		void solve(const double dt)
		{
			const double dta = dt / _steps;
			for (int i = 0; i<_steps; i++) { _solver->solve(dta); }
		}

	private:

		GISolver _solver;
		double _steps;
	};



	class GISolverFourthOrder : public GISolverBase
	{
	public:
		GISolverFourthOrder(GISolver& s) :
			_solver(s)
		{
			_a = 1.0 / (2.0 - std::pow(2.0, 1.0 / 3.0));
			_b = 1.0 - 2.0*_a;
		}

		~GISolverFourthOrder() {}

		void init() { _solver->init(); }

		void solve(const double dt)
		{
			const double dta = _a * dt;
			const double dtb = _b * dt;
			_solver->solve(dta);
			_solver->solve(dtb);
			_solver->solve(dta);
		}


	private:

		GISolver _solver;
		double _a, _b;
	};


	class GISolverSixthOrder : public GISolverBase
	{
	public:
		GISolverSixthOrder(GISolver& s) :
			_solver(s)
		{
			_a = 1.0 / (4.0 - std::pow(4.0, 1.0 / 3.0));
			_b = 1.0 - 4.0*_a;
		}

		~GISolverSixthOrder() {}

		void init() { _solver->init(); }

		void solve(const double dt)
		{
			const double dta = _a * dt;
			const double dtb = _b * dt;
			_solver->solve(dta);
			_solver->solve(dta);
			_solver->solve(dtb);
			_solver->solve(dta);
			_solver->solve(dta);
		}


	private:

		GISolver _solver;
		double _a, _b;
	};

	/*
	class GISolverIterated : public GISolverBase
	{
	public:
	GISolverIterated( GISolver& s, int n = 1 ) :
	_solver (s)
	{
	_n = n;
	int k = 2;
	GISolverIterated test = dynamic_cast<GISolverIterated>(s);
	if( test ){ k = test->_order; }
	_a = 1.0/( (double)(2*n) - std::pow((double)2*n, 1.0/(double)(k+1)) );
	_b = 1.0 - 2.0*n*_a;
	_order = k+2;
	}

	~GISolverIterated(){}

	void init(){ _solver->init(); }

	void solve( const double dt )
	{
	const double dta = _a * dt;
	const double dtb = _b * dt;
	for( int i=0;i<_n;i++ )
	{
	_solver->solve(dta);
	}
	_solver->solve(dtb);
	for( int i=0;i<_n;i++ )
	{
	_solver->solve(dta);
	}
	}


	private:

	GISolver _solver;
	double _a, _b;
	int _order;
	int _n;
	};
	*/


	class LeapFrogSolver : public GISolverBase
	{
	public:

		LeapFrogSolver(GISolver& A, GISolver&  B) :
			a(A),
			b(B)
		{}

		~LeapFrogSolver() {}

		void init() { a->init(); b->init(); }

		void solve(const double dt)//dt = delta t = 全局的计时器单位，一步的长度
		{
			const double dtd2 = 0.5*dt;
			a->solve(dtd2);//a 是private，GISolver里面定义的，但是这只是类中的定义，在我
			//们使用的实际对象中，他已经在bouncingball的构造函数中被赋值了，成为了advanced position/velocity
			b->solve(dt);
			a->solve(dtd2);
		}

	private:

		GISolver a;
		GISolver b;
	};


	class ForwardEulerSolver : public GISolverBase
	{
	public:

		ForwardEulerSolver(GISolver& A, GISolver& B) :
			a(A),
			b(B)
		{}

		~ForwardEulerSolver() {}

		void init() { a->init(); b->init(); }

		void solve(const double dt)
		{
			a->solve(dt);
			b->solve(dt);
		}

	private:

		GISolver a;
		GISolver b;
	};


	// From eqn (11) of "Geometric Integrators for ODEs"
	// by McLachlan and Quispel
	class BlanesMoanSolver : public GISolverBase
	{
	public:

		BlanesMoanSolver(GISolver& A, GISolver& B) :
			a(A),
			b(B)
		{
			_a[0] = 0.0792036964311957;
			_a[1] = 0.353172906049774;
			_a[2] = -0.0420650803577195;
			_a[3] = 1.0 - 2.0*(_a[0] + _a[1] + _a[2]);
			_a[4] = _a[2];
			_a[5] = _a[1];
			_a[6] = _a[0];

			_b[0] = 0; // not used
			_b[1] = 0.209515106613362;
			_b[2] = -0.143851773179818;
			_b[3] = 0.5 - (_b[1] + _b[2]);
			_b[4] = _b[3];
			_b[5] = _b[2];
			_b[6] = _b[0];
		}

		~BlanesMoanSolver() {}

		void init() { a->init(); b->init(); }

		void solve(const double dt)
		{
			a->solve(_a[0] * dt);
			for (int i = 1; i <= 6; i++)
			{
				b->solve(_b[i] * dt);
				a->solve(_a[i] * dt);
			}

		}

	private:

		GISolver a;
		GISolver b;
		double _a[7], _b[7];
	};



	GISolver CreateGISolverSubstep(GISolver& s, int nbsteps);
	GISolver CreateGISolverFourthOrder(GISolver& s);
	GISolver CreateGISolverSixthOrder(GISolver& s);
	GISolver CreateLeapFrogSolver(GISolver& A, GISolver&  B);
	GISolver CreateForwardEulerSolver(GISolver& A, GISolver& B);
	GISolver CreateBlanesMoanSolver(GISolver& A, GISolver& B);









}


#endif