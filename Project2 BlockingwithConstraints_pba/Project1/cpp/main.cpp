
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
	//PbaThing cube = PbaThing(new RotatingCubeThing());//һ���µ�pba��������
	//PbaThing ball = PbaThing(new BouncingBallsThing());//ע��=BouncingBalls

	PbaThing ponSphere = PbaThing(new ParticleOnASphereThing(0.1, 0.8, 0.8, 1, 5.0, 1.0, 5.0));
	//PbaThing aBasicBoidthing = PbaThing(new BasicBoidThing(0.1, 1.0, 1.0, 20, 2.0, 2.0, 9.0 ));

	// collision surface
	/*CollisionSurface surf = buildcube();
	std::shared_ptr<BasicBoidThing> boidf= dynamic_pointer_cast<BasicBoidThing>(aBasicBoidthing);
	boidf->AddCollisionSurface(surf);*/


	//viewer->AddThing(cube);
	//viewer->AddThing(aBasicBoidthing);//���뵽viewer֮�д洢�������Ա���
	viewer->AddThing(ponSphere);

	std::vector<string> vec;//string���͵�vector <>?
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
//int main()
//{
//	return 1;
//}