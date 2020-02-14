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
#include "SphinATeapot.h"
#include "ObjParser.h"


int main(int argc, char *argv[])
{
	using namespace pba;
	PbaViewer* viewer = PbaViewer::Instance();
	//PbaThing cube = PbaThing(new RotatingCubeThing());//一个新的pba类别的物体
	//PbaThing ball = PbaThing(new BouncingBallsThing());//注释=BouncingBalls

	if (false)
	{
		/*PbaThing ponSphere = PbaThing(new ParticleOnASphereThing(0.1, 0.8, 0.8, 1, 5.0, 1.0, 5.0));
		viewer->AddThing(ponSphere);*/
	}
	//PbaThing aBasicBoidthing = PbaThing(new BasicBoidThing(0.1, 1.0, 1.0, 20, 2.0, 2.0, 9.0 ));
	//viewer->AddThing(aBasicBoidthing);//加入到viewer之中存储起来，以便检测

	// collision surface
	/*CollisionSurface surf = buildcube();
	std::shared_ptr<BasicBoidThing> boidf= dynamic_pointer_cast<BasicBoidThing>(aBasicBoidthing);
	boidf->AddCollisionSurface(surf);*/

	//viewer->AddThing(cube);
	

	if (true)
	{
		SphInATeapotThing *Sph = new SphInATeapotThing("d");

		ObjParser* objReader = new ObjParser();
		objReader->ParseFile("C:\\Users\\Administrator\\Desktop\\teapot.obj");
		CollisionSurface surf = makeCollisionSurface();
		objReader->Fill(surf);
		//surf->toggle_wireframe();

		Sph->AddCollisionSurface(surf);
		PbaThing Sphthing = PbaThing(Sph);
		viewer->AddThing(Sphthing);
		/*float a = 0.0;
		Vector v = Vector(0.0, 0.0, 0.0);
		float t = v.magnitude();*/
	}
	

	std::vector<string> vec;//string类型的vector <>
	for (int i=0; i<argc; i++)
	{
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	return 0;//不会执行
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
