#pragma once
//-------------------------------------------------------
//
//  BasicBoid.h
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
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "ForceLibrary.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
#include "ParticleEmitter.h"
//#include "PbaUtils.h"

#include "Mymethods.h"

#include <cstdlib>

#ifdef __APPLE__
#include <OpenGL/gl.h>   /// OpenGL itself.
#include <OpenGL/glu.h>  /// GLU support library.
#include <GLUT/glut.h>
#else
#include <GL/gl.h>   /// OpenGL itself.
#include <GL/glu.h>  /// GLU support library.
#include <GL/glut.h> /// GLUT support library.
#endif

#include <iostream>



using namespace std;

namespace pba {




	class BasicBoidThing : public PbaThingyDingy
	{
	public:

		BasicBoidThing(const double avoid, const double match, const double center, const int nb, const double amax, const double rng, const double rng_ramp, const std::string nam = "BasicBoidThing") :
			PbaThingyDingy(nam),
			box(pba::makeCollisionSurface())
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->add(nb);

			boid_force = CreateAccumulatingBoidForce(avoid, match, center, amax, rng, rng_ramp);
			//harmonic_force = CreateAccumulatingHarmonicForce(0.1, Vector(0, 0, 0));
			force = CreateAccumulatingForce();

			std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(force);
			//...
			std::shared_ptr<AccumulatingBoidForce> boidf = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
			boidf->set_lead_boid(0);
			//...
			f->add(boid_force);
			///f->add(harmonic_force);

			///GISolver solvera = CreateAdvancePosition( state );
			GISolver solvera = CreateAdvancePosition(state, collisions);
			GISolver solverb = CreateAdvanceVelocity(state, force);
			///solver = CreateForwardEulerSolver(solvera,solverb);
			solver = CreateLeapFrogSolver(solvera, solverb);
			///solver = CreateGISolverSixthOrder(solver);
			///solver = CreateRK4Solver(state,force);
			Reset();
			std::cout << name << " constructed\n";
		};
		~BasicBoidThing() {};

		void Init(const std::vector<std::string>& args)
		{
		}

		/// Callback functions
		void Display()
		{
			//pba::Display(box);

			glPointSize(5.0);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < state->nb(); i++)
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
			if (key == 'W') { box->toggle_visible(); }
			if (key == 'w') { box->toggle_wireframe(); }
			if (key == 'k') { collisions.toggle_tree(); }
			if (key == 'a')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_avoidance(f->get_avoidance() / 1.1);
				std::cout << "Collision Avoidance Strength: " << f->get_avoidance() << std::endl;
			}
			if (key == 'A')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_avoidance(f->get_avoidance()*1.1);
				std::cout << "Collision Avoidance Strength: " << f->get_avoidance() << std::endl;
			}
			if (key == 'v')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_matching(f->get_matching() / 1.1);
				std::cout << "Velocity Matching Strength: " << f->get_matching() << std::endl;
			}
			if (key == 'V')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_matching(f->get_matching()*1.1);
				std::cout << "Velocity Matching Strength: " << f->get_matching() << std::endl;
			}
			if (key == 'c')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_centering(f->get_centering() / 1.1);
				std::cout << "Centering Strength: " << f->get_centering() << std::endl;
			}
			if (key == 'C')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_centering(f->get_centering()*1.1);
				std::cout << "Centering Strength: " << f->get_centering() << std::endl;
			}
			if (key == 'm')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_max(f->get_max() / 1.1);
				std::cout << "Maximum Acceleration: " << f->get_max() << std::endl;
			}
			if (key == 'M')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_max(f->get_max()*1.1);
				std::cout << "Maximum Acceleration: " << f->get_max() << std::endl;
			}
			if (key == 'd')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_range(f->get_range() / 1.1);
				std::cout << "Range: " << f->get_range() << std::endl;
			}
			if (key == 'D')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_range(f->get_range()*1.1);
				std::cout << "Range: " << f->get_range() << std::endl;
			}
			if (key == 'y')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_range_ramp(f->get_range_ramp() / 1.1);
				std::cout << "Range Ramp: " << f->get_range_ramp() << std::endl;
			}
			if (key == 'Y')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_range_ramp(f->get_range_ramp()*1.1);
				std::cout << "Range Ramp: " << f->get_range_ramp() << std::endl;
			}
			if (key == 'q')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_fov(f->get_fov() / 1.1);
				std::cout << "FOV: " << f->get_fov() << std::endl;
			}
			if (key == 'Q')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_fov(f->get_fov()*1.1);
				std::cout << "FOV: " << f->get_fov() << std::endl;
			}
		};
		void solve()
		{
			solver->solve(dt);
		};
		void Reset()
		{
			/// Distribute particles with random positions
			for (size_t i = 0; i < state->nb(); i++)
			{
				Vector P(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
				P *= 2.0;
				state->set_pos(i, P);
				Vector V(0.1, 0.1, 0.1);
				state->set_vel(i, V);
				Color C(drand48(), drand48(), drand48(), 0.0);
				state->set_ci(i, C);
			}
		};
		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "W            toggle visibility of collision surface\n";
			cout << "w            toggle wireframe of collision surface\n";
			cout << "k            toggle collision trace tree on/off\n";
			cout << "a/A          reduce/increase collision avoidance force constant\n";
			cout << "v/V          reduce/increase velocity matching force constant\n";
			cout << "c/C          reduce/increase centering force constant\n";
			cout << "m/M          reduce/increase maximum acceleration\n";
			cout << "d/D          reduce/increase range\n";
			cout << "y/Y          reduce/increase range ramp\n";
			cout << "q/Q          reduce/increase boid fov\n";
		};


		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			collisions.set_collision_surface(box);
		}



	private:

		pba::DynamicalState state;
		pba::Force boid_force;
		pba::Force harmonic_force;
		pba::Force force;
		pba::GISolver solver;

		pba::CollisionSurface box;
		pba::ElasticCollisionHandler collisions;


	};



	pba::PbaThing BasicBoid(const double avoid, const double match, const double center, const int nb, const double amax, const double range, const double range_ramp)
	{
		return PbaThing(new BasicBoidThing(avoid, match, center, nb, amax, range, range_ramp));
	}



}





