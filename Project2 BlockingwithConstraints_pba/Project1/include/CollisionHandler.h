#pragma once
//*******************************************************************
//
//   CollisionHandler.h
//
// Process collisions
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//*******************************************************************

#ifndef __PBA_COLLISIONHANDLER_H__
#define __PBA_COLLISIONHANDLER_H__

#include "CollisionSurface.h"
#include "DynamicalState.h"
//#include "RigidBodyState.h"
//#include "SoftBodyState.h"
//#include "TraceTree.h"
#include <iostream>

namespace pba {

	// Base class for collision handling
	class CollisionHandler
	{
	public:

		/*CollisionHandler() : surf(0), tree(0), usetree(false) {}
		virtual ~CollisionHandler() { if (tree) { delete tree; } }*/

		virtual void handle_collisions(const double dt, DynamicalState& s) { std::cout << "CollisionHandler::handle_collisions(double,DynamicalState) called\n"; };
		/*virtual void handle_collisions(const double dt, RigidBodyState& s) { std::cout << "CollisionHandler::handle_collisions(double,DynamicalState) called\n"; };
		virtual void handle_collisions(const double dt, SoftBodyState& s) { std::cout << "CollisionHandler::handle_collisions(double,DynamicalState) called\n"; };*/

		void set_collision_surface(CollisionSurface& c);

		//是否使用树结构组织数据
		void use_tree() { usetree = true; }
		void dont_use_tree() { usetree = false; }
		void toggle_tree() { usetree = !usetree; }

	protected:

		CollisionSurface surf;
		//TraceTree* tree;
		bool usetree;


	};



	class ElasticCollisionHandler : public CollisionHandler
	{
	public:

		ElasticCollisionHandler()
		{};

		~ElasticCollisionHandler() {}

		void handle_collisions(const double dt, DynamicalState& s);

	};


	//class ElasticSphereCollisionHandler : public CollisionHandler
	//{
	//public:

	//	ElasticSphereCollisionHandler() : do_self_collisions(false)
	//	{};

	//	~ElasticSphereCollisionHandler() {}

	//	void handle_collisions(const double dt, DynamicalState& s);

	//	bool do_self_collisions;

	//};


	//class ElasticSphereSphereCollisionHandler : public CollisionHandler
	//{
	//public:

	//	ElasticSphereSphereCollisionHandler() : do_self_collisions(false)
	//	{};

	//	~ElasticSphereSphereCollisionHandler() {}

	//	void handle_collisions(const double dt, DynamicalState& s);

	//	bool do_self_collisions;

	//};




	//// This is just collisions of the particles in the RBD, not any facets or edges.
	//class ElasticRBDCollisionHandler : public CollisionHandler
	//{
	//public:

	//	ElasticRBDCollisionHandler() :
	//		coeff_of_restitution(1.0)
	//	{};

	//	~ElasticRBDCollisionHandler() {}

	//	//void handle_collisions(const double dt, RigidBodyState& s);

	//	void set_CR(const float v) { coeff_of_restitution = v; }

	//private:

	//	float coeff_of_restitution;

	//};


	//// This is just collisions of the particles in the RBD, not any facets or edges.
	//class ElasticSBDCollisionHandler : public CollisionHandler
	//{
	//public:

	//	ElasticSBDCollisionHandler() :
	//		coeff_of_restitution(1.0)
	//	{};

	//	~ElasticSBDCollisionHandler() {}

	//	//void handle_collisions(const double dt, SoftBodyState& s);

	//	void set_CR(const float v) { coeff_of_restitution = v; }

	//private:

	//	float coeff_of_restitution;

	//};








}

#endif
