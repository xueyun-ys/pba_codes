
#include <Windows.h>
#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>
#include "RotatingCube.h"
#include "BouncingBalls.h"
#include "PbaViewer.h"
#include <string>
#include <vector>
#include "ParticleOnASphere.h"
#include "BasicBoid.h"
#include "Mymethods.h"

int main(int argc, char *argv[])
{
	using namespace pba;
	PbaViewer* viewer = PbaViewer::Instance();
	//PbaThing cube = PbaThing(new RotatingCubeThing());//一个新的pba类别的物体
	//PbaThing ball = PbaThing(new BouncingBallsThing());//注释=BouncingBalls

	PbaThing ponSphere = PbaThing(new ParticleOnASphereThing(0.1, 0.8, 0.8, 1, 5.0, 1.0, 5.0));
	//PbaThing aBasicBoidthing = PbaThing(new BasicBoidThing(0.1, 1.0, 1.0, 20, 2.0, 2.0, 9.0 ));

	// collision surface
	/*CollisionSurface surf = buildcube();
	std::shared_ptr<BasicBoidThing> boidf= dynamic_pointer_cast<BasicBoidThing>(aBasicBoidthing);
	boidf->AddCollisionSurface(surf);*/


	//viewer->AddThing(cube);
	//viewer->AddThing(aBasicBoidthing);//加入到viewer之中存储起来，以便检测
	viewer->AddThing(ponSphere);

	std::vector<string> vec;//string类型的vector <>?
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
//int main()
//{
//	return 1;
//}