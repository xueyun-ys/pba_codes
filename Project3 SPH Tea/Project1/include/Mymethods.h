#pragma once

#include <iostream>
#include "CollisionSurface.h"

using namespace std;

namespace pba {
	
	double drand48();

	CollisionSurface buildcube();

	void Display2(pba::CollisionSurface s);

	void AddCollisionSurface(pba::CollisionSurface& s);
}