
#include <Windows.h>
#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>
#include "RotatingCube.h"
#include "BouncingBalls.h"
#include "PbaViewer.h"
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
	using namespace pba;
	PbaViewer* viewer = PbaViewer::Instance();
	PbaThing cube = PbaThing(new RotatingCubeThing());//一个新的pba类别的物体
	PbaThing ball = PbaThing(new BouncingBallsThing());//注释=BouncingBalls

	//viewer->AddThing(cube);
	viewer->AddThing(ball);//加入到viewer之中存储起来，以便检测

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