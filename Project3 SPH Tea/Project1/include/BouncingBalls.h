//-------------------------------------------------------
//
//  BouncingBalls.h
//
//  PbaThing for a collection of particles with gravity 
//  and a collision surface
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#pragma once 

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "ForceLibrary.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
//#include "ParticleEmitter.h"
//#include "PbaUtils.h"

#include <cstdlib>

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
#include <OpenGL/glu.h>  // GLU support library.
#include <GLUT/glut.h>
#else
#include <Windows.h>
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
//#include <GL/glew.h>

#endif

#include <iostream>

using namespace std;

namespace pba {


	class BouncingBallsThing : public PbaThingyDingy
	{

	public:

		/*RotatingCubeThing(const std::string nam = "RotatingCubeThing") :
			PbaThingyDingy(nam),
			axis(pba::Vector(0, 1, 0)),
			theta(0.0),
			dtheta(24.0*3.14159265 / 180.0)
		{
			std::cout << name << " constructed\n";
		};
		~RotatingCubeThing() {};*/












		//! Initialization, including GLUT initialization.
		void Init(const std::vector<std::string>& args)
		{
			verts[0] = pba::Vector(-1, -1, -1);
			verts[1] = pba::Vector(1, -1, -1);
			verts[2] = pba::Vector(1, 1, -1);
			verts[3] = pba::Vector(-1, 1, -1);
			verts[4] = pba::Vector(-1, -1, 1);
			verts[5] = pba::Vector(1, -1, 1);
			verts[6] = pba::Vector(1, 1, 1);
			verts[7] = pba::Vector(-1, 1, 1);

			normals[0] = pba::Vector(1, 0, 0);
			normals[1] = pba::Vector(0, 1, 0);
			normals[2] = pba::Vector(0, 0, 1);
			normals[3] = pba::Vector(-1, 0, 0);
			normals[4] = pba::Vector(0, -1, 0);
			normals[5] = pba::Vector(0, 0, -1);

			face_colors[0] = pba::Color(1, 0, 1, 0);
			face_colors[1] = pba::Color(1, 0, 0, 0);
			face_colors[2] = pba::Color(0, 0, 1, 0);
			face_colors[3] = pba::Color(0, 1, 0, 0);
			face_colors[4] = pba::Color(1, 1, 0, 0);
			face_colors[5] = pba::Color(0, 1, 1, 0);

			std::vector<int> face;
			face.push_back(1);
			face.push_back(2);
			face.push_back(6);
			face.push_back(5);
			faces.push_back(face);

			face[0] = 2;
			face[1] = 3;
			face[2] = 7;
			face[3] = 6;
			faces.push_back(face);

			/*face[0] = 0;
			face[1] = 3;
			face[2] = 2;
			face[3] = 1;
			faces.push_back(face);*/

			face[0] = 0;
			face[1] = 4;
			face[2] = 7;
			face[3] = 3;
			faces.push_back(face);

			face[0] = 0;
			face[1] = 1;
			face[2] = 5;
			face[3] = 4;
			faces.push_back(face);

			face[0] = 5;
			face[1] = 6;
			face[2] = 7;
			face[3] = 4;
			faces.push_back(face);


		////==========================================================
			CollisionSurface surf = makeCollisionSurface();//新建对象

			for (size_t i = 0; i < faces.size(); i++)
			{
				std::vector<int>& face = faces[i];

				CollisionTriangle tri1 = makeCollisionTriangle(Vertex(face[0]), Vertex(face[1]), Vertex(face[2]));//构造new一个新的collision triangle对象
				tri1->set_color(face_colors[i]);
				surf->addTriangle(tri1);

				CollisionTriangle tri2 = makeCollisionTriangle(Vertex(face[2]), Vertex(face[3]), Vertex(face[0]));
				int index = (i + 1) % 6;
				tri2->set_color(face_colors[(i + 1) % 6]);
				surf->addTriangle(tri2);
			}

			AddCollisionSurface(surf);

		}


		//! Cascading callback for an idle  event 
		//void solve() { theta += dtheta*dt; };
		//! Cascading callback for reseting parameters
		//void Reset() { theta = 0.0; };
		//! Cascading callback for usage information
		/*void Usage()
		{
			PbaThingyDingy::Usage();
		};*/

		//! Vertex after rotation
		const pba::Vector Vertex(const int i) const
		{
			int ii = i % 8;
			pba::Vector result = verts[ii].rotate(axis, theta);
			return result;
			//return verts[i];
		}

		//! Normal after rotation
		const pba::Vector Normal(const int i) const
		{
			int ii = i % 6;
			pba::Vector result = normals[ii].rotate(axis, theta);
			return result;
		}


		//=====================================================================================================================================================================





		/*BouncingBallsThing(const std::string nam = "RotatingCubeThing") :
			PbaThingyDingy(nam),
			axis(pba::Vector(0, 1, 0)),
			theta(0.0),
			dtheta(24.0*3.14159265 / 180.0)
		{
			std::cout << name << " constructed\n";
		};
		~BouncingBallsThing() {};*/




		BouncingBallsThing(const std::string nam = "BouncingBallsThing") ://构造函数~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~·2.5初始化
			PbaThingyDingy(nam),
			axis(pba::Vector(0, 1, 0)),
			theta(0.0),
			dtheta(24.0*3.14159265 / 180.0),
			emit(false)//,
			//box(pba::makeCollisionSurface()),
			//emitter(ParticleEmitter(Vector(0, 0, 0), Vector(0, 0, 0), 0.25, 1.0))
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->add(1);
			force = CreateSimpleGravityForce(pba::Vector(0, -0.1f, 0));
			GISolver solvera = CreateAdvancePosition(state, collisions);//将state 传入 pq,collision传入cs;A是advancedPwithC
			//返回advancepositionwithcollision?
			GISolver solverb = CreateAdvanceVelocity(state, force);//B=advanced velocity
			//返回advancevelocity?pq, force
			////solver = CreateForwardEulerSolver(solvera,solverb);
			solver = CreateLeapFrogSolver(solvera, solverb);//a, b存入leapfrog solver里面
			//solver = CreateGISolverSixthOrder(solver);
			Reset();
			std::cout << name << " constructed\n";
		};
		~BouncingBallsThing() {};

		

		// Callback functions
		//! Cascading callback for initiating a display event
		void Display()
		{
			
		
			//glBegin(GL_QUADS);
			//glBegin(GL_TRIANGLES);
			for (size_t i = 0; i < faces.size(); i++)
			{
				glBegin(GL_TRIANGLES);

				glColor3f(face_colors[i].red(), face_colors[i].green(), face_colors[i].blue());
				std::vector<int>& face = faces[i];

				// 1
				pba::Vector v = Vertex(face[0]);
				glVertex3f(v.X(), v.Y(), v.Z());
				v = Normal(face[0]);
				glNormal3f(v.X(), v.Y(), v.Z());

				// 2
				v = Vertex(face[1]);
				glVertex3f(v.X(), v.Y(), v.Z());
				v = Normal(face[1]);
				glNormal3f(v.X(), v.Y(), v.Z());

				// 3
				v = Vertex(face[2]);
				glVertex3f(v.X(), v.Y(), v.Z());
				v = Normal(face[2]);
				glNormal3f(v.X(), v.Y(), v.Z());
				//glEnd();

				glColor3f(face_colors[i + 1].red(), face_colors[i + 1].green(), face_colors[i + 1].blue());
				//glBegin(GL_TRIANGLES);
				// 3
				v = Vertex(face[2]);
				glVertex3f(v.X(), v.Y(), v.Z());
				v = Normal(face[2]);
				glNormal3f(v.X(), v.Y(), v.Z());

				// 4
				v = Vertex(face[3]);
				glVertex3f(v.X(), v.Y(), v.Z());
				v = Normal(face[3]);
				glNormal3f(v.X(), v.Y(), v.Z());

				// 1
				v = Vertex(face[0]);
				glVertex3f(v.X(), v.Y(), v.Z());
				v = Normal(face[0]);
				glNormal3f(v.X(), v.Y(), v.Z());

				glEnd();
			}
			//glEnd();
			
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

			//==========================================================
			/*CollisionSurface surf = makeCollisionSurface();

			for (size_t i = 0; i < faces.size(); i++)
			{
				std::vector<int>& facetemp = faces[i];

				CollisionTriangle tri1 = makeCollisionTriangle(Vertex(facetemp[0]), Vertex(facetemp[1]), Vertex(facetemp[2]));
				tri1->set_color(face_colors[i]);
				surf->addTriangle(tri1);

				CollisionTriangle tri2 = makeCollisionTriangle(Vertex(facetemp[2]), Vertex(facetemp[3]), Vertex(facetemp[0]));
				int index = (i + 1) % 6;
				tri2->set_color(face_colors[(i + 1) % 6]);
				surf->addTriangle(tri2);
			}

			AddCollisionSurface(surf);*/







		};

		//void Keyboard(unsigned char key, int x, int y)
		//{
		//	PbaThingyDingy::Keyboard(key, x, y);
		//	if (key == 'v') { box->toggle_visible(); }
		//	if (key == 'w') { box->toggle_wireframe(); }
		//	if (key == 'e') { emit = !emit; }
		//	if (key == 'g')
		//	{
		//		std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(force);
		//		f->set_gravity_constant(f->get_gravity_constant() / 1.1);
		//	}
		//	if (key == 'G')
		//	{
		//		std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(force);
		//		f->set_gravity_constant(f->get_gravity_constant()*1.1);
		//	}
		//	if (key == 'k') { collisions.toggle_tree(); }
		//	if (key == 'c')
		//	{
		//		box->set_coeff_restitution(box->coeff_restitution() / 1.1);
		//		std::cout << "coefficient of restituion: " << box->coeff_restitution() << std::endl;
		//	}
		//	if (key == 'C')
		//	{
		//		box->set_coeff_restitution(box->coeff_restitution()*1.1);
		//		std::cout << "coefficient of restituion: " << box->coeff_restitution() << std::endl;
		//	}
		//	if (key == 's')
		//	{
		//		box->set_coeff_sticky(box->coeff_sticky() / 1.1);
		//		std::cout << "coefficient of sticky: " << box->coeff_sticky() << std::endl;
		//	}
		//	if (key == 'S')
		//	{
		//		box->set_coeff_sticky(box->coeff_sticky()*1.1);
		//		std::cout << "coefficient of sticky: " << box->coeff_sticky() << std::endl;
		//	}
		//	if (key == 'l')
		//	{
		//		GISolver solvera = CreateAdvancePosition(state, collisions);
		//		GISolver solverb = CreateAdvanceVelocity(state, force);
		//		//solver = CreateForwardEulerSolver(solverb,solvera); //backward
		//		//solver = CreateForwardEulerSolver(solvera,solverb); // forward
		//		solver = CreateLeapFrogSolver(solvera, solverb);
		//		//solver = CreateGISolverSixthOrder(solver);
		//		std::cout << "Using Leap Frog solver" << std::endl;
		//	}
		//	if (key == 'f')
		//	{
		//		GISolver solvera = CreateAdvancePosition(state, collisions);
		//		GISolver solverb = CreateAdvanceVelocity(state, force);
		//		//solver = CreateForwardEulerSolver(solverb,solvera); //backward
		//		solver = CreateForwardEulerSolver(solvera, solverb); // forward
		//															 //solver = CreateLeapFrogSolver(solvera,solverb);
		//															 //solver = CreateGISolverSixthOrder(solver);
		//		std::cout << "Using Forward Euler solver" << std::endl;
		//	}
		//	if (key == 'b')
		//	{
		//		GISolver solverb = CreateAdvanceVelocity(state, force);
		//		GISolver solvera = CreateAdvancePosition(state, collisions);
		//		solver = CreateForwardEulerSolver(solverb, solvera); //backward
		//															 //solver = CreateForwardEulerSolver(solvera,solverb); // forward
		//															 //solver = CreateLeapFrogSolver(solvera,solverb);
		//															 //solver = CreateGISolverSixthOrder(solver);
		//		std::cout << "Using Backward Euler solver" << std::endl;
		//	}
		//	if (key == '6')
		//	{
		//		GISolver solvera = CreateAdvancePosition(state, collisions);
		//		GISolver solverb = CreateAdvanceVelocity(state, force);
		//		//solver = CreateForwardEulerSolver(solverb,solvera); //backward
		//		//solver = CreateForwardEulerSolver(solvera,solverb); // forward
		//		solver = CreateLeapFrogSolver(solvera, solverb);
		//		solver = CreateGISolverSixthOrder(solver);
		//		std::cout << "Using Sixth Order solver" << std::endl;
		//	}
		//};
		void solve() //3.程序正式开始
		{
			//if (emit)
			//{
			//	int nbincrease = 10;
			//	state->add(nbincrease);
			//	Vector P, V;
			//	Color C;
			//	std::cout << "Emit Points " << state->nb() << std::endl;
			//	for (size_t i = state->nb() - nbincrease; i<state->nb(); i++)
			//	{
			//		emitter.emit(P, V, C);
			//		state->set_pos(i, P);
			//		state->set_vel(i, V);
			//		state->set_ci(i, C);
			//	}
			//}
			solver->solve(dt);//boucing ball 有个pba::GISolver solver;dt在pbadything中声明的变量，这个B也是智能指针【??????
			//这里因为在构造函数里已经把它转换为了leapfrog类型了
		};
		void Reset()
		{
			// Distribute particles with random positions
			Vector P, V;
			Color C;
			//for (size_t i = 0; i<state->nb(); i++)
			//{
			//	emitter.emit(P, V, C);
			//	state->set_pos(i, P);
			//	state->set_vel(i, V);
			//	state->set_ci(i, C);
			//}
		};
		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "v            toggle visibility of collision surface\n";
			cout << "w            toggle wireframe/solid display of collision surface\n";
			cout << "g/G          reduce/increase gravitational constant\n";
			cout << "e            toggle particle emission on/off\n";
			cout << "k            toggle collision trace tree on/off\n";
			cout << "c/C          reduce/increase coefficient of restitution\n";
			cout << "l            use leap frog solver\n";
			cout << "f            use forward euler solver\n";
			cout << "b            use backward euler solver\n";
			cout << "6            use sixth order sover\n";

		};

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			collisions.set_collision_surface(box);//box存入collision handler 中的collision_surface变量中
		}

		/*void addCollisionSurface(CollisionSurface& surf)
		{
			box = surf;
			collisions.set_collision_surface(box);
		}*/


	private:

		// rotation axis
		pba::Vector axis;
		double theta;
		double dtheta;

		// vertices
		pba::Vector verts[8];

		// face normals
		pba::Vector normals[6];

		// faces
		std::vector< std::vector<int> > faces;

		// face colors
		pba::Color face_colors[6];



		bool emit;

		pba::DynamicalState state;
		pba::Force force;
		pba::GISolver solver;

		pba::CollisionSurface box;
		pba::ElasticCollisionHandler collisions;
		//pba::ParticleEmitter emitter;

	};



	pba::PbaThing BouncingBalls() { return PbaThing(new BouncingBallsThing()); }



}


