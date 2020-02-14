
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
	PbaThing cube = PbaThing(new RotatingCubeThing());//һ���µ�pba��������
	PbaThing ball = PbaThing(new BouncingBallsThing());//ע��=BouncingBalls

	//viewer->AddThing(cube);
	viewer->AddThing(ball);//���뵽viewer֮�д洢�������Ա���

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