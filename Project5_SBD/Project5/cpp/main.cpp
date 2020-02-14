#include <Windows.h>
#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>
//#include "RotatingCube.h"
//#include "BouncingBalls.h"
#include "PbaViewer.h"
#include <string>
#include <vector>
//#include "ParticleOnASphere.h"
//#include "BasicBoid.h"
#include "Mymethods.h"
//#include "SphinATeapot.h"
#include "ObjParser.h"
//#include "BouncingRBD.h"
#include "SBDAreaCloth.h"


int main(int argc, char *argv[])
{
	using namespace pba;
	PbaViewer* viewer = PbaViewer::Instance();
	//PbaThing cube = PbaThing(new RotatingCubeThing());//һ���µ�pba��������
	//PbaThing ball = PbaThing(new BouncingBallsThing());//ע��=BouncingBalls

	if (false)
	{
		/*PbaThing ponSphere = PbaThing(new ParticleOnASphereThing(0.1, 0.8, 0.8, 1, 5.0, 1.0, 5.0));
		viewer->AddThing(ponSphere);*/
	}
	//PbaThing aBasicBoidthing = PbaThing(new BasicBoidThing(0.1, 1.0, 1.0, 20, 2.0, 2.0, 9.0 ));
	//viewer->AddThing(aBasicBoidthing);//���뵽viewer֮�д洢�������Ա���

	// collision surface
	/*CollisionSurface surf = buildcube();
	std::shared_ptr<BasicBoidThing> boidf= dynamic_pointer_cast<BasicBoidThing>(aBasicBoidthing);
	boidf->AddCollisionSurface(surf);*/

	//viewer->AddThing(cube);
	

	if (true)
	{
		SBDAreaClothThing *sbd = new SBDAreaClothThing("s");

		//ObjParser* objReader = new ObjParser();
		//objReader->ParseFile("C:\\Users\\Administrator\\Desktop\\bunny.obj");
		//CollisionSurface surf = makeCollisionSurface();
		//objReader->Fill(surf);
		//surf->toggle_wireframe();

		//rbd->AddCollisionSurface(surf);
		PbaThing Sphthing = PbaThing(sbd);
		viewer->AddThing(Sphthing);
		/*float a = 0.0;
		Vector v = Vector(0.0, 0.0, 0.0);
		float t = v.magnitude();*/

		//create box
		CollisionSurface csBox = buildcube(0.5);
		sbd->AddCollisionSurface(csBox);
		/*CollisionSurface csGrd = buildground(1.0);
		sbd->AddCollisionSurface(csGrd);*/
	}


	std::vector<string> vec;//string���͵�vector <>
	for (int i=0; i<argc; i++)
	{
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	return 0;//����ִ��
}

//#include <iostream>
//

	/*CollisionSurface cube = pba::GenerateCollisionCube(1.0);
	{
		cube->get_triangle(4)->set_invisable();
		cube->get_triangle(5)->set_invisable();
	}*/

	

	/*SphInATeapotThing* BouncingBalls = new SphInATeapotThing;
	BouncingBalls->AddCollisionSurface(surf);
	PbaThing balls = PbaThing(BouncingBalls);

	viewer->AddThing(balls);*/
	
	//std::cout << "Hello" << std::endl;
