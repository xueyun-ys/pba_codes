#pragma once

#include <iostream>
#include "CollisionSurface.h"

using namespace std;

namespace pba {
	
	double drand48();

	CollisionSurface buildcube(double scale);

	CollisionSurface buildground(double scale);

	void Display(pba::CollisionSurface s);

	void AddCollisionSurface(pba::CollisionSurface& s);

}