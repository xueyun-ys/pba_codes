#pragma once
//-------------------------------------------------------
//
//  SphInATeapot.h
//
//  PbaThing for a collection of sph particles with gravity 
//  and a collision surface
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
#include "SPHSolver.h"
#include "RK4.h"
#include "ForceLibrary.h"
#include "SPHForce.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
#include "ParticleEmitter.h"
#include "PbaUtils.h"

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

#include "Mymethods.h"

using namespace std;

namespace pba {




	class SphInATeapotThing : public PbaThingyDingy
	{
	public:

		SphInATeapotThing(const std::string nam = "SphInATeapotThing") :
			PbaThingyDingy(nam),
			emit(false),
			box(pba::makeCollisionSurface()),
			emitter(ParticleEmitter(Vector(50.5, -10.0, 1.5), Vector(-1.0, 0, 0), 2.5, 10.0))
		{
			//AABB bounds(Vector(-90, -40, -60), Vector(100, 50, 56));
			double h = 90.0 / 200.0;
			state = CreateSPH(/*bounds,*/ h, name + "DynamicalData");
			//state->add(280000/10);

			force = CreateAccumulatingForce();
			//gravityforce = CreateAccumulatingGravityForce(pba::Vector(0, -100.0, 0));
			gravityforce = CreateSimpleGravityForce(pba::Vector(0, -100.0, 0));

			sphforce = CreateSPHForce();

			std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(force);//����ת������force�ࣨ���ࣩת��ΪAccumulatingForce����ٵ��������add�����������������;
			f->add(gravityforce);//f�����洢������Ȼ��ֱ�compute��ͨ���Լ���compute����
			f->add(sphforce);
			solvera = CreateAdvancePosition(state, collisions);
			solverb = CreateAdvanceVelocity(state, force, 100, 100);//500000000.6, 1200000000000.0);
			solver = CreateForwardEulerSolver(solvera, solverb);
			//solver = CreateLeapFrogSolver(solvera,solverb);
			//solver = CreateGISolverSixthOrder(solver);
			Reset();
			std::cout << name << " constructed\n";
		};
		~SphInATeapotThing() {};

		void Init(const std::vector<std::string>& args)
		{
			SetSimulationTimestep(1.0 / 48.0);
		}

		// Callback functions
		void Display()
		{
			pba::Display(box);

			glPointSize(3.5);
			glBegin(GL_POINTS);
			for (size_t i = 0; i<state->nb(); i++)
			{
				const Color& ci = state->ci(i);
				const pba::Vector& v = state->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glVertex3f(v.X(), v.Y(), v.Z());
			}
			glEnd();
		};

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'v') { box->toggle_visible(); }
			if (key == 'w') { box->toggle_wireframe(); }
			if (key == 'e') { emit = !emit; }
			if (key == 'g')
			{
				//std::shared_ptr<AccumulatingGravityForce> f = dynamic_pointer_cast<AccumulatingGravityForce>(gravityforce);
				std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(gravityforce);
				f->set_gravity_constant(f->get_gravity_constant() / 1.1);
				std::cout << "Gravity strength: " << f->get_gravity_constant() << std::endl;//�иĶ�
			}
			if (key == 'G')
			{
				//std::shared_ptr<AccumulatingGravityForce> f = dynamic_pointer_cast<AccumulatingGravityForce>(gravityforce);
				std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(gravityforce);
				f->set_gravity_constant(f->get_gravity_constant()*1.1);
				std::cout << "Gravity strength: " << f->get_gravity_constant() << std::endl;
			}
			if (key == 'p')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_pressure_magnitude(f->get_pressure_magnitude() / 1.1);
				std::cout << "Pressure magnitude: " << f->get_pressure_magnitude() << std::endl;
			}
			if (key == 'P')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_pressure_magnitude(f->get_pressure_magnitude()*1.1);
				std::cout << "Pressure magnitude: " << f->get_pressure_magnitude() << std::endl;
			}
			if (key == 'y')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_pressure_base(f->get_pressure_base() / 1.1);
				std::cout << "Pressure base: " << f->get_pressure_base() << std::endl;
			}
			if (key == 'Y')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_pressure_base(f->get_pressure_base()*1.1);
				std::cout << "Pressure base: " << f->get_pressure_base() << std::endl;
			}
			if (key == 'j')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_pressure_power(f->get_pressure_power() / 1.1);
				std::cout << "Pressure power: " << f->get_pressure_power() << std::endl;
			}
			if (key == 'J')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_pressure_power(f->get_pressure_power()*1.1);
				std::cout << "Pressure power: " << f->get_pressure_power() << std::endl;
			}
			if (key == 'a')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_alpha_sph(f->get_alpha_sph() / 1.1);
				std::cout << "Viscosity alpha: " << f->get_alpha_sph() << std::endl;
			}
			if (key == 'A')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_alpha_sph(f->get_alpha_sph()*1.1);
				std::cout << "Viscosity alpha: " << f->get_alpha_sph() << std::endl;
			}
			if (key == 'b')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_beta_sph(f->get_beta_sph() / 1.1);
				std::cout << "Viscosity beta: " << f->get_beta_sph() << std::endl;
			}
			if (key == 'B')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_beta_sph(f->get_beta_sph()*1.1);
				std::cout << "Viscosity beta: " << f->get_beta_sph() << std::endl;
			}
			if (key == 'q')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_epsilon_sph(f->get_epsilon_sph() / 1.1);
				std::cout << "Viscosity epsilon: " << f->get_epsilon_sph() << std::endl;
			}
			if (key == 'Q')
			{
				std::shared_ptr<SPHForce> f = dynamic_pointer_cast<SPHForce>(sphforce);
				f->set_epsilon_sph(f->get_epsilon_sph()*1.1);
				std::cout << "Viscosity epsilon: " << f->get_epsilon_sph() << std::endl;
			}
			if (key == 'i')
			{
				state->set_radius(state->get_radius() / 1.1);
				std::cout << "Radius: " << state->get_radius() << std::endl;
			}
			if (key == 'I')
			{
				state->set_radius(state->get_radius()*1.1);
				std::cout << "Radius: " << state->get_radius() << std::endl;
			}
			if (key == 'o')
			{
				std::shared_ptr<AdvanceSPHVelocity> f = dynamic_pointer_cast<AdvanceSPHVelocity>(solverb);
				f->set_velocity_clamp(f->get_velocity_clamp() / 1.1);
				std::cout << "Velocity clamp: " << f->get_velocity_clamp() << std::endl;
			}
			if (key == 'O')
			{
				std::shared_ptr<AdvanceSPHVelocity> f = dynamic_pointer_cast<AdvanceSPHVelocity>(solverb);
				f->set_velocity_clamp(f->get_velocity_clamp()*1.1);
				std::cout << "Velocity clamp: " << f->get_velocity_clamp() << std::endl;
			}
			if (key == 'm')
			{
				std::shared_ptr<AdvanceSPHVelocity> f = dynamic_pointer_cast<AdvanceSPHVelocity>(solverb);
				f->set_acceleration_clamp(f->get_acceleration_clamp() / 1.1);
				std::cout << "Acceleration clamp: " << f->get_acceleration_clamp() << std::endl;
			}
			if (key == 'M')
			{
				std::shared_ptr<AdvanceSPHVelocity> f = dynamic_pointer_cast<AdvanceSPHVelocity>(solverb);
				f->set_acceleration_clamp(f->get_acceleration_clamp()*1.1);
				std::cout << "Acceleration clamp: " << f->get_acceleration_clamp() << std::endl;
			}


			if (key == 'k') { collisions.toggle_tree(); }
			if (key == 'c')
			{
				box->set_coeff_restitution(box->coeff_restitution() / 1.1);
				std::cout << "coefficient of restituion: " << box->coeff_restitution() << std::endl;
			}
			if (key == 'C')
			{
				box->set_coeff_restitution(box->coeff_restitution()*1.1);
				std::cout << "coefficient of restituion: " << box->coeff_restitution() << std::endl;
			}
			if (key == 's')
			{
				box->set_coeff_sticky(box->coeff_sticky() / 1.1);
				std::cout << "coefficient of sticky: " << box->coeff_sticky() << std::endl;
			}
			if (key == 'S')
			{
				box->set_coeff_sticky(box->coeff_sticky()*1.1);
				std::cout << "coefficient of sticky: " << box->coeff_sticky() << std::endl;
			}
			if (key == 'l')
			{
				//solver = CreateForwardEulerSolver(solverb,solvera); //backward
				//solver = CreateForwardEulerSolver(solvera,solverb); // forward
				solver = CreateLeapFrogSolver(solvera, solverb);
				//solver = CreateGISolverSixthOrder(solver);
				std::cout << "Using Leap Frog solver" << std::endl;
			}
			if (key == 'z')
			{
				//solver = CreateForwardEulerSolver(solverb,solvera); //backward
				solver = CreateForwardEulerSolver(solvera, solverb); // forward
																	 //solver = CreateLeapFrogSolver(solvera,solverb);
																	 //solver = CreateGISolverSixthOrder(solver);
				std::cout << "Using Forward Euler solver" << std::endl;
			}
			if (key == 'x')
			{
				solver = CreateForwardEulerSolver(solverb, solvera); //backward
																	 //solver = CreateForwardEulerSolver(solvera,solverb); // forward
																	 //solver = CreateLeapFrogSolver(solvera,solverb);
																	 //solver = CreateGISolverSixthOrder(solver);
				std::cout << "Using Backward Euler solver" << std::endl;
			}
			if (key == '6')
			{
				//solver = CreateForwardEulerSolver(solverb,solvera); //backward
				//solver = CreateForwardEulerSolver(solvera,solverb); // forward
				solver = CreateLeapFrogSolver(solvera, solverb);
				solver = CreateGISolverSixthOrder(solver);
				std::cout << "Using Sixth Order solver" << std::endl;
			}
		};
		void solve()
		{
			if (emit)
			{
				int nbincrease = 1;// 0000;
				state->add(nbincrease);
				std::cout << "Emit: Total Points " << state->nb() << std::endl;
#pragma omp parallel for
				for (size_t i = state->nb() - nbincrease; i<state->nb(); i++)
				{
					Vector P, V;
					Color C;
#pragma omp critical
					{
						emitter.emit(P, V, C);
					}
					C[0] = 0.8 + 0.2*C[0];
					state->set_pos(i, P);
					state->set_vel(i, V);
					state->set_ci(i, C);
				}
				emit = false;
			}
			solver->solve(dt);
		};
		void Reset()
		{
			/*
			Vector P, V;
			Color C;
			// Dam Break
			V = Vector(0.0,0.0,0.0);
			for(size_t i=0;i<state->nb();i++)
			{
			P = Vector( 2.0*drand48()-2.99, 3.5*drand48()-2.99, -2.0*drand48()+2.99 );
			C = Color( drand48(), drand48(), 0.2*drand48() + 0.8, 1.0 );

			state->set_pos(i, P);
			state->set_vel(i, V);
			state->set_ci(i, C);
			}
			*/
			//state->clear();//???????

		};
		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "v            toggle visibility of collision surface\n";
			cout << "w            toggle wireframe/solid display of collision surface\n";
			cout << "g/G          reduce/increase gravitational constant\n";
			cout << "p/P          reduce/increase pressure magnitude\n";
			cout << "y/Y          reduce/increase pressure base\n";
			cout << "j/J          reduce/increase pressure power\n";
			cout << "a/A          reduce/increase viscosity alpha\n";
			cout << "b/B          reduce/increase viscosity beta\n";
			cout << "q/Q          reduce/increase viscosity epsilon\n";
			cout << "i/I          reduce/increase occupancy volume cellsize\n";
			cout << "o/O          reduce/increase velocity clamp\n";
			cout << "m/M          reduce/increase acceleration clamp\n";
			cout << "e            toggle particle emission on/off\n";
			cout << "k            toggle collision trace tree on/off\n";
			cout << "c/C          reduce/increase coefficient of restitution\n";
			cout << "s/S          reduce/increase coefficient of stickiness\n";
			cout << "l            use leap frog solver\n";
			cout << "z            use forward euler solver\n";
			cout << "x            use backward euler solver\n";
			cout << "6            use sixth order solver\n";

		};

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			//std::cout << "AABB: [" << s->aabb().LLC().X() << ", " << s->aabb().LLC().Y() << ", " << s->aabb().LLC().Z() << "] X [" << s->aabb().URC().X() << ", " << s->aabb().URC().Y() << ", " << s->aabb().URC().Z() << "]" << std::endl;
			box = s;
			collisions.set_collision_surface(box);
		}


	private:

		bool emit;

		pba::SPHState state;
		pba::Force force;
		pba::Force sphforce;
		pba::Force gravityforce;
		pba::GISolver solver;
		pba::GISolver solvera;
		pba::GISolver solverb;

		pba::CollisionSurface box;
		pba::ElasticCollisionHandler collisions;
		pba::ParticleEmitter emitter;

		float vclamp;

	};



	pba::PbaThing SphInATeapot() { return PbaThing(new SphInATeapotThing()); }



}





