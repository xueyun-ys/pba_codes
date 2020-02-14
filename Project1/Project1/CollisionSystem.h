#pragma once
//*******************************************************************
//
//   CollisionSystem.h
//
// Class for collisions between dynamical state objects
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//*******************************************************************

#ifndef __PBA_COLLISIONSYSTEM_H__
#define __PBA_COLLISIONSYSTEM_H__

#include "DynamicalState.h"
#include <memory>

namespace pba {

	struct CollisionSystemEvent
	{
		double t;
		size_t i;
		size_t j;
		bool status;
	};




	class CollisionSystemRaw
	{
	public:

		CollisionSystemRaw(DynamicalState& s);
		~CollisionSystemRaw() {}

		// self-collision of spheres
		bool sphere_hit(double tmax, CollisionSystemEvent& event);

		void set_visible() { visible = true; }
		void set_invisible() { visible = false; }
		bool visibility() const { return visible; }


	private:

		DynamicalState state;
		bool visible;

	};

	typedef std::shared_ptr<CollisionSystemRaw> CollisionSystem;

	CollisionSystem makeCollisionSystem(DynamicalState& s);

}

#endif
