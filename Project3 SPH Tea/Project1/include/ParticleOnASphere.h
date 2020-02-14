#pragma once
//-------------------------------------------------------
//
//  ParticleOnASphere.h
//
//  PbaThing for a collection of particles with gravity 
//  and a collision surface
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "ConstraintLibrary.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "ForceLibrary.h"
//#include "PoincareData.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
#include "ParticleEmitter.h"
//#include "PbaUtils.h"
#include "Mymethods.h"

#include <cstdlib>

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
#include <OpenGL/glu.h>  // GLU support library.
#include <GLUT/glut.h>
#else
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#endif

#include <iostream>



using namespace std;

namespace pba {




	class ParticleOnASphereThing : public PbaThingyDingy
	{
	public:

		//ParticleOnASphereThing(const std::string nam = "ParticleOnASphereThing") :
		ParticleOnASphereThing(const double avoid, const double match, const double center, const int nb, const double amax, const double rng, const double rng_ramp, const std::string nam = "ParticleOnASphereThing") :
			PbaThingyDingy(nam),
			constraint(pba::CreateMultiConstraint()),
			emitter(ParticleEmitter(Vector(0, 1, 0), Vector(0, 0, 0), 0.01, 0.1)),
			emit(false),
			constraint_is_visible(true),
			report(false)
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->add(nb);

			for (size_t i = 0; i < state->nb(); i++)
			{
				Constraint pcon = CreateParticleOnSphereConstraint(1.0, Vector(0, 0, 0), i);
				pcon->set_Ks(0.0000001);
				pcon->set_Kf(0.0000001);
				constraint->addConstraint(pcon);
			}
#define _CONST_AND_BOID
#ifdef _CONST_AND_BOID
			//ѡ��ʹ��������������������������flocking������һ����..��
			//force = CreateSimpleGravityForce(pba::Vector(0, -1.0, 0));
			boid_force = CreateAccumulatingBoidForce(avoid, match, center, amax, rng, rng_ramp);
			// set lead boid
			{
				leadBoidID = 0;
				std::shared_ptr<AccumulatingBoidForce> boidF = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				boidF->set_lead_boid(leadBoidID);
			} 
			force = CreateAccumulatingForce();
			std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(force);
			f->add(boid_force);
#else
			force = CreateSimpleGravityForce(pba::Vector(0, -1.0, 0));
#endif 

			//��һ���ֶ��൱�ڳ�ʼ����Ϊִֻ��һ�Σ������ͨ�����ּ�123���޸�solver
			/// These two partial solvers show the relaxation method.
			/// All of the constraint treatment is in the force, 
			/// including the "relaxed" constraint.
			//GISolver solvera = CreateAdvancePosition(state);
			//GISolver solverb = CreateAdvanceVelocity(state, force, constraint);

			/// These two solvers show an almost "original" PBD
			//GISolver solvera = CreateAdvancePosition( state, constraint );
			//GISolver solverb = CreateAdvanceVelocity(state, force);

			/// These two solvers show PBD plus relaxation
			GISolver solvera = CreateAdvancePosition( state, constraint );
			GISolver solverb = CreateAdvanceVelocity(state, force, constraint);

			solver = CreateLeapFrogSolver(solvera, solverb);
			///solver = CreateGISolverSixthOrder(solver);
			Reset();
			std::cout << name << " constructed\n";
		};
		~ParticleOnASphereThing() {};

		void Init(const std::vector<std::string>& args)
		{



		}

		/// Callback functions
		void Display()
		{
			/// pba::Display(box);

			if (constraint_is_visible)
			{
				glColor3f(1.0, 1.0, 1.0);
				glutWireSphere(1.0, 20, 20);
			}

			glPointSize(10.0);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < state->nb(); i++)
			{
				const Color& ci = state->ci(i);
				const pba::Vector& v = state->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glVertex3f(v.X(), v.Y(), v.Z());
			}
			glEnd();

			if (report)
			{
				for (size_t i = 0; i < state->nb(); i++)
				{
					std::cout << "Magnitude for particle " << i << "   " << state->pos(i).magnitude() << std::endl;
				}
			}
		};

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'v') { constraint_is_visible = !constraint_is_visible; }
			///if( key == 'w' ){ box->toggle_wireframe(); }
			if (key == 'R') { report = !report; }
			if (key == 'e') { emit = !emit; }
			if (key == 'g')
			{
				std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(force);
				f->set_gravity_constant(f->get_gravity_constant() / 1.1);
			}
			if (key == 'G')
			{
				std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(force);
				f->set_gravity_constant(f->get_gravity_constant()*1.1);
			}
			if (key == 's')
			{
				double Ks = constraint->get_constraint(0)->get_Ks();
				Ks -= 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Ks(Ks);
				}
				std::cout << "Constraint Ks " << Ks << std::endl;
			}
			if (key == 'S')
			{
				double Ks = constraint->get_constraint(0)->get_Ks();
				Ks += 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Ks(Ks);
				}
				std::cout << "Constraint Ks " << Ks << std::endl;
			}
			if (key == 'd')
			{
				double Kf = constraint->get_constraint(0)->get_Kf();
				Kf -= 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Kf(Kf);
				}
				std::cout << "Constraint Kf " << Kf << std::endl;
			}
			if (key == 'D')
			{
				double Kf = constraint->get_constraint(0)->get_Kf();
				Kf += 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Kf(Kf);
				}
				std::cout << "Constraint Kf " << Kf << std::endl;
			}
			if (key == '1')
			{
				GISolver solvera = CreateAdvancePosition(state);
				GISolver solverb = CreateAdvanceVelocity(state, force, constraint);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Relaxation only " << std::endl;
			}
			if (key == '2')
			{
				GISolver solvera = CreateAdvancePosition(state, constraint);
				GISolver solverb = CreateAdvanceVelocity2(state, force, constraint);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Position-Based Only " << std::endl;
				/*GISolver solvera = CreateAdvancePosition(state, constraint);
				GISolver solverb = CreateAdvanceVelocity(state, force);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Position-Based Only " << std::endl;*/
			}
			if (key == '3')
			{
				GISolver solvera = CreateAdvancePosition(state, constraint);
				GISolver solverb = CreateAdvanceVelocity(state, force, constraint);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Mixed Position-Based and Relaxation " << std::endl;
			}






		};
		void solve()
		{
			if (emit)
			{
				int nbincrease = 10;// 0;
				state->add(nbincrease);
				Vector P, V;
				Color C;
				std::cout << "Emit Points " << state->nb() << std::endl;
				double Ks = constraint->get_constraint(0)->get_Ks();
				double Kf = constraint->get_constraint(0)->get_Kf();
				for (size_t i = state->nb() - nbincrease; i < state->nb(); i++)
				{
					C = Color(drand48(), drand48(), drand48(), 1);
					double s = 2.0 * drand48() - 1.0;
					double phi = drand48() * 2.0 * 3.14159265;
					double ss = std::sqrt(1.0 - s * s);
					P = Vector(ss * std::cos(phi), s - 0.1, ss * std::sin(phi));
					P = P.unitvector();
					V = Vector(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
					V -= ((V * P) / (P * P)) * P;
					V *= (1.0 + drand48() * 3.0) / 2.0;


					//emitter.emit(P, V, C);
					state->set_pos(i, P);
					state->set_vel(i, V);
					state->set_ci(i, C);

					Constraint pcon = CreateParticleOnSphereConstraint(1.0, Vector(0, 0, 0), (int)i);//�뾶�����ģ�ID  
					pcon->set_Ks(Ks);
					pcon->set_Kf(Kf);
					constraint->addConstraint(pcon);
					
					emit = false;
				}
			}
			solver->solve(dt);//���յ�solver����ʾ���������������������solver
		};
		void Reset()
		{
			/// Distribute particles with random positions
			for (size_t i = 0; i < state->nb(); i++)
			{
				Color C(drand48(), drand48(), drand48(), 1);
				//Color C(0.5, 0.5, 0.5, 1);
				double s = 2.0*drand48() - 1.0;
				double phi = drand48()*2.0*3.14159265;
				double ss = std::sqrt(1.0 - s*s);
				Vector P(ss*std::cos(phi), s, ss*std::sin(phi));
				P = P.unitvector();
				Vector V(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
				V -= ((V*P) / (P*P))*P;
				V *= (1.0 + drand48()*3.0) / 2.0;
				state->set_pos(i, P);
				state->set_vel(i, V);
				state->set_ci(i, C);
			}
		};
		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "v            toggle visibility of collision surface\n";
			///cout << "w            toggle wireframe/solid display of collision surface\n";
			cout << "e            toggle emitting particles\n";
			cout << "g/G          reduce/increase gravitational constant\n";
			cout << "s/S          reduce/increase constraint spring force\n";
			cout << "d/D          reduce/increase constraint friction force\n";
			cout << "R            toggle reporting particle data on/off\n";
			cout << "1            Relaxation solver\n";
			cout << "2            Position-based solver\n";
			cout << "3            Mixed relaxation & position-based solver\n";
		};


	private:

		pba::DynamicalState state;
		pba::Force force;
		pba::GISolver solver;
		pba::MultiConstraint constraint;
		//pba::Constraint constraint;
		pba::ParticleEmitter emitter;
		bool emit;
		bool constraint_is_visible;


		bool report;


		pba::Force boid_force;
		int leadBoidID;

	};



	//pba::PbaThing ParticleOnASphere() { return PbaThing(new ParticleOnASphereThing()); }



}