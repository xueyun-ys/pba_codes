#include "Mymethods.h"
#include "Vector.h"
#include "Color.h"

#include "PbaThing.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
#include "ForceLibrary.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
//#include "ParticleEmitter.h"
//#include "BasicBoid.h"

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



double pba::drand48()
{
	return (double)rand() / RAND_MAX;
}

using namespace std;
namespace pba 
{
	const pba::Vector Vertex(const int i)
	{
		int ii = i % 8;
		pba::Vector result = verts[ii].rotate(axis, theta);
		return result;
		//return verts[i];
	}

	//! Normal after rotation
	const pba::Vector Normal(const int i) //const
	{
		int ii = i % 6;
		pba::Vector result = normals[ii].rotate(axis, theta);
		return result;
	}



	pba::CollisionSurface buildcube()
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

		return surf;
		//AddCollisionSurface(surf);
	}

	
	void Display2(pba::CollisionSurface s)
	{
		if (s->use_wireframe())
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_TRIANGLES);

		glBegin(GL_TRIANGLES);
		for (size_t i = 0; i < s->triangle_size(); i++)
		{
			if (!s->get_triangle(i)->visibility())
				continue;

			pba::Color color = s->get_triangle(i)->get_color();
			glColor4f(color.red(), color.green(), color.blue(), color.alpha());

			pba::Vector v = s->get_triangle(i)->vertex(0);
			glVertex3f(v.X(), v.Y(), v.Z());
			glNormal3f(v.X(), v.Y(), v.Z());

			v = s->get_triangle(i)->vertex(1);
			glVertex3f(v.X(), v.Y(), v.Z());
			glNormal3f(v.X(), v.Y(), v.Z());

			v = s->get_triangle(i)->vertex(2);
			glVertex3f(v.X(), v.Y(), v.Z());
			glNormal3f(v.X(), v.Y(), v.Z());
		}
		glEnd();
		return;
	}

	void AddCollisionSurface(pba::CollisionSurface& s)
	{
		std::cout << "Add CollisionSurface\n";
		box = s;
		collisions.set_collision_surface(box);
	}

}

