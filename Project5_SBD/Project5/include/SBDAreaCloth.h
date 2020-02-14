#pragma once
//-------------------------------------------------------
//
//  SBDAreaCloth.h
//
//  PbaThing for a collection of particles with gravity 
//  and a collision surface, joined as a SBD object
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "SoftBodyState.h"
#include "SBDSolver.h"
#include "RK4.h"
#include "ForceLibrary.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
#include "PbaUtils.h"
#include "LinearAlgebra.h"
#include "ObjParser.h"
#include "PbaViewer.h"

#include <cstdlib>

#include "Mymethods.h"

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
#include <sstream>



using namespace std;

namespace pba {




	class SBDAreaClothThing : public PbaThingyDingy
	{
	public:

		SBDAreaClothThing(const std::string& fs, const std::string nam = "SBDAreaClothThing") :
			PbaThingyDingy(nam),
			box(pba::makeCollisionSurface()),
			initial_velocity(pba::Vector(0.0, 0.0, 0.0)),
			initial_position(pba::Vector(0.0, 0.0, 0.0)),
			track_sbd(true)
		{
			std::cout << "Constructing " << fs << std::endl;
			state = CreateSoftBody(name + "SoftBodyData");

			AddStateGeometry(pba::Vector(-2, 1.5, -2), pba::Vector(2, 1.5, 2), 30, 30);

			force = CreateAccumulatingForce();
			gforce = CreateAccumulatingGravityForce(pba::Vector(0, -2, 0));//-2
			strutforce = CreateAccumulatingStrutForce(1400.0, 1.0);//14000
			strutareaforce = CreateAccumulatingStrutAreaForce(1500.0, 26.0);//15000
			strutbendforce = CreateAccumulatingStrutBendForce(15.0, 30.0);
			std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(force);//f=force....................?
			f->add(strutforce);//add force into forces
			f->add(strutareaforce);
			f->add(strutbendforce);
			f->add(gforce);
			std::cout << "Forces constructed\n";


			GISolver solvera = CreateAdvanceRotation(state, collisions);
			GISolver solverb = CreateAdvanceAngularVelocity(state, force);
			basicsolver = CreateLeapFrogSolver(solvera, solverb);
			///basicsolver = CreateGISolverSixthOrder(basicsolver);
			solver = CreateGISolverSubstep(basicsolver, 9);//9

			std::cout << name << " constructed\n";
		};
		~SBDAreaClothThing() {};

		void Init(const std::vector<std::string>& args)
		{
			std::cout << "Init " << name << std::endl;
			///toggleAnimate();
			collisions.toggle_tree();
		}

		// Callback functions
		void Display()
		{
			pba::Display(box);

			glBegin(GL_LINES);//draw the edges between two particles
			const Color ci(1.0, 1.0, 1.0, 1.0);
			glColor3f(ci.red(), ci.green(), ci.blue());
			for (size_t i = 0; i<state->nb_pairs(); i++)
			{
				const pba::SoftEdge& e = state->get_connected_pair(i);
				const pba::Vector& v1 = state->pos(e->get_first_node());
				glVertex3f(v1.X(), v1.Y(), v1.Z());
				const pba::Vector& v2 = state->pos(e->get_second_node());
				glVertex3f(v2.X(), v2.Y(), v2.Z());
			}
			glEnd();

		};

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'W') { box->toggle_visible(); }
			if (key == 'w') { box->toggle_wireframe(); }
			if (key == 'K') { track_sbd = !track_sbd; }
			if (key == 'k')
			{
				collisions.toggle_tree();
				std::cout << "Toggled collision tree\n";
			}
			if (key == 'g')
			{
				std::shared_ptr<AccumulatingGravityForce> f = dynamic_pointer_cast<AccumulatingGravityForce>(gforce);
				f->set_strength(f->get_strength() / 1.1);
				std::cout << "Gravity strength " << f->get_strength() << std::endl;
			}
			if (key == 'G')
			{
				std::shared_ptr<AccumulatingGravityForce> f = dynamic_pointer_cast<AccumulatingGravityForce>(gforce);
				f->set_strength(f->get_strength()*1.1);
				std::cout << "Gravity strength " << f->get_strength() << std::endl;
			}

			if (key == '1' || key == '2' || key == '3' || key == '4' || key == '5' || key == '6' || key == '7' || key == '8' || key == '9')
			{
				solver = CreateGISolverSubstep(basicsolver, int(key) - 48);
				std::cout << "Switched to solver with " << int(key) - 48 << " substeps\n";
			}

			if (key == 's')
			{
				std::shared_ptr<AccumulatingStrutForce> f = dynamic_pointer_cast<AccumulatingStrutForce>(strutforce);
				f->set_strength(f->get_strength() / 1.1);
				std::cout << "Strut strength " << f->get_strength() << std::endl;
			}
			if (key == 'S')
			{
				std::shared_ptr<AccumulatingStrutForce> f = dynamic_pointer_cast<AccumulatingStrutForce>(strutforce);
				f->set_strength(f->get_strength()*1.1);
				std::cout << "Strut strength " << f->get_strength() << std::endl;
			}
			if (key == 'v')
			{
				std::shared_ptr<AccumulatingStrutForce> f = dynamic_pointer_cast<AccumulatingStrutForce>(strutforce);
				f->set_friction(f->get_friction() / 1.1);
				std::cout << "Strut friction " << f->get_friction() << std::endl;
			}
			if (key == 'V')
			{
				std::shared_ptr<AccumulatingStrutForce> f = dynamic_pointer_cast<AccumulatingStrutForce>(strutforce);
				f->set_friction(f->get_friction()*1.1);
				std::cout << "Strut friction " << f->get_friction() << std::endl;
			}
			if (key == 'a')
			{
				std::shared_ptr<AccumulatingStrutAreaForce> f = dynamic_pointer_cast<AccumulatingStrutAreaForce>(strutareaforce);
				f->set_strength(f->get_strength() / 1.1);
				std::cout << "Strut Area strength " << f->get_strength() << std::endl;
			}
			if (key == 'A')
			{
				std::shared_ptr<AccumulatingStrutAreaForce> f = dynamic_pointer_cast<AccumulatingStrutAreaForce>(strutareaforce);
				f->set_strength(f->get_strength()*1.1);
				std::cout << "Strut Area strength " << f->get_strength() << std::endl;
			}
			if (key == 'b')
			{
				std::shared_ptr<AccumulatingStrutAreaForce> f = dynamic_pointer_cast<AccumulatingStrutAreaForce>(strutareaforce);
				f->set_friction(f->get_friction() / 1.1);
				std::cout << "Strut Area friction " << f->get_friction() << std::endl;
			}
			if (key == 'B')
			{
				std::shared_ptr<AccumulatingStrutAreaForce> f = dynamic_pointer_cast<AccumulatingStrutAreaForce>(strutareaforce);
				f->set_friction(f->get_friction()*1.1);
				std::cout << "Strut Area friction " << f->get_friction() << std::endl;
			}
			if (key == 'n')
			{
				std::shared_ptr<AccumulatingStrutBendForce> f = dynamic_pointer_cast<AccumulatingStrutBendForce>(strutbendforce);
				f->set_strength(f->get_strength() / 1.1);
				std::cout << "Strut Bend strength " << f->get_strength() << std::endl;
			}
			if (key == 'N')
			{
				std::shared_ptr<AccumulatingStrutBendForce> f = dynamic_pointer_cast<AccumulatingStrutBendForce>(strutbendforce);
				f->set_strength(f->get_strength()*1.1);
				std::cout << "Strut Bend strength " << f->get_strength() << std::endl;
			}
			if (key == 'm')
			{
				std::shared_ptr<AccumulatingStrutBendForce> f = dynamic_pointer_cast<AccumulatingStrutBendForce>(strutbendforce);
				f->set_friction(f->get_friction() / 1.1);
				std::cout << "Strut Bend friction " << f->get_friction() << std::endl;
			}
			if (key == 'M')
			{
				std::shared_ptr<AccumulatingStrutBendForce> f = dynamic_pointer_cast<AccumulatingStrutBendForce>(strutbendforce);
				f->set_friction(f->get_friction()*1.1);
				std::cout << "Strut Bend friction " << f->get_friction() << std::endl;
			}


		};
		void solve()
		{
			solver->solve(dt);
			/*if (track_sbd && viewer != 0)
			{
				Vector cen = geometric_center(state);
				viewer->SetCameraEyeViewUp(cen.X(), cen.Y(), cen.Z() - 15.0, cen.X(), cen.Y(), cen.Z(), 0.0, 1.0, 0.0);
			}*/
		};
		void Reset()
		{
			///state->clear();
			state->clear_pairs();
			state->add(home_state->nb());

			for (size_t i = 0; i<home_state->nb(); i++)
			{
				state->set_ci(i, home_state->ci(i));
				state->set_pos(i, home_state->pos(i));
				state->set_vel(i, home_state->vel(i));
			}
			for (size_t i = 0; i<home_state->nb_pairs(); i++)
			{
				const SoftEdge& e = home_state->get_connected_pair(i);
				state->add_pair(e->get_first_node(), e->get_second_node());
			}
			for (size_t i = 0; i<home_state->nb_area_sets(); i++)
			{
				const SoftTriangle& e = home_state->get_area_set(i);
				state->add_triangle(e->get_first_node(), e->get_second_node(), e->get_third_node());
			}
			for (size_t i = 0; i<home_state->nb_bendables(); i++)
			{
				const SoftBendable& e = home_state->get_bendable(i);
				state->add_bend(e->get_first_node(), e->get_second_node(), e->get_third_node(), e->get_fourth_node());
			}
			std::cout << "Nb points " << state->nb() << "    Nb pairs " << state->nb_pairs() << "    Nb Area Sets " << state->nb_area_sets() << "   Nb bend pairs " << state->nb_bendables() << std::endl;
		};

		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "p            toggle between normal display and Poincare plot\n";
			cout << "W            toggle visibility of collision surface\n";
			cout << "w            toggle wireframe/solid display of collision surface\n";
			cout << "g/G          reduce/increase gravity strength\n";
			cout << "s/S          reduce/increase strut strength\n";
			cout << "v/V          reduce/increase strut friction strength\n";
			cout << "a/A          reduce/increase strut area strength\n";
			cout << "b/B          reduce/increase strut area friction\n";
			cout << "n/N          reduce/increase strut bend strength\n";
			cout << "m/M          reduce/increase strut bend friction\n";
			cout << "1-9          select number of sover substeps\n";
			cout << "k            toggle collision trace tree on/off\n";
			cout << "K            toggle tracking the sbd object on/off\n";
		};

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			collisions.set_collision_surface(box);
		};

		void AddStateGeometry(const std::string& f)
		{
			std::cout << "Importing state geometry " << f << std::endl;
			home_state = CreateSoftBody(name + "home_data");
			ObjParser parser;
			///parser.ParseFile(f);
			///parser.Fill(home_state);
			std::cout << "nb verts " << home_state->nb() << std::endl;
			DynamicalState ds = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(home_state);
			///AABB home_aabb = pba::BoundingBox(ds);
			//Vector cen = (home_aabb.URC() + home_aabb.LLC())*0.5;
			//pba::Scale( cen, pba::Vector(4.0,4.0,4.0), ds);
			//pba::Translate( -cen, ds);
			//home_aabb = pba::BoundingBox(ds);
			///std::cout << "home_aabb: " << home_aabb.LLC().X() << " " << home_aabb.LLC().Y() << " " << home_aabb.LLC().Z() << "  X  " << home_aabb.URC().X() << " " << home_aabb.URC().Y() << " " << home_aabb.URC().Z() << std::endl;
			//pba::Translate( initial_position-(home_aabb.URC()-home_aabb.LLC()), ds );
			for (size_t i = 0; i<home_state->nb(); i++)
			{
				Color C(drand48(), drand48(), drand48(), 1.0);
				home_state->set_ci(i, C);
				home_state->set_vel(i, initial_velocity);
			}
			Reset();
		};

		void AddStateGeometry(const pba::Vector& LLC, const pba::Vector& URC, int nx, int ny)
		{
			std::cout << "ASG start\n";
			home_state = GeneratePlanarSoftBody(LLC, URC, nx, ny);
			for (size_t i = 0; i<home_state->nb(); i++)
			{
				Color C(drand48(), drand48(), drand48(), 1.0);
				home_state->set_ci(i, C);
				home_state->set_vel(i, initial_velocity);
			}
			std::cout << "ASG reset\n";
			Reset();
			std::cout << "ASG end\n";
		};


		std::map<std::string, std::string> MetaData() const
		{
			std::stringstream dtstream;
			dtstream << dt;
			std::map<std::string, std::string> md;
			md[name + ":dt"] = dtstream.str();

			std::shared_ptr<AccumulatingGravityForce> f0 = dynamic_pointer_cast<AccumulatingGravityForce>(gforce);
			std::stringstream gstream;
			gstream << f0->get_strength();
			md[name + ":Gravity"] = gstream.str();


			std::shared_ptr<AccumulatingStrutForce> f1 = dynamic_pointer_cast<AccumulatingStrutForce>(strutforce);
			std::stringstream stutstrengthstream;
			stutstrengthstream << f1->get_strength();
			md[name + ":StrutStrength"] = stutstrengthstream.str();

			std::stringstream stutfrictionstream;
			stutfrictionstream << f1->get_friction();
			md[name + ":StrutFriction"] = stutfrictionstream.str();

			std::shared_ptr<AccumulatingStrutAreaForce> f2 = dynamic_pointer_cast<AccumulatingStrutAreaForce>(strutareaforce);
			std::stringstream stutareastrengthstream;
			stutareastrengthstream << f2->get_strength();
			md[name + ":StrutAreaStrength"] = stutareastrengthstream.str();

			std::stringstream stutareafrictionstream;
			stutareafrictionstream << f2->get_friction();
			md[name + ":StrutAreaFriction"] = stutareafrictionstream.str();

			std::shared_ptr<AccumulatingStrutBendForce> f3 = dynamic_pointer_cast<AccumulatingStrutBendForce>(strutbendforce);
			std::stringstream stutbendstrengthstream;
			stutbendstrengthstream << f3->get_strength();
			md[name + ":StrutBendStrength"] = stutbendstrengthstream.str();

			std::stringstream stutbendfrictionstream;
			stutbendfrictionstream << f3->get_friction();
			md[name + ":StrutBendFriction"] = stutbendfrictionstream.str();


			return md;
		}


	private:

		pba::SoftBodyState state;
		pba::SoftBodyState home_state;
		pba::Force force;
		pba::Force gforce;
		pba::Force strutforce;
		pba::Force strutareaforce;
		pba::Force strutbendforce;
		pba::GISolver basicsolver;
		pba::GISolver solver;

		pba::CollisionSurface box;
		pba::ElasticSBDCollisionHandler collisions;

		pba::Vector initial_velocity;
		pba::Vector initial_position;

		bool track_sbd;
	};



	pba::PbaThing SBDAreaCloth(const std::string& f) { return PbaThing(new SBDAreaClothThing(f)); }


}




