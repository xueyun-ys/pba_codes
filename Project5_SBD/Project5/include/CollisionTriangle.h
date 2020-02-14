#pragma once
//*******************************************************************
//
//   CollisionTriangle.h
//
// Class for a triangle for collisions
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//*******************************************************************

#ifndef __PBA_COLLISIONTRIANGLE_H__
#define __PBA_COLLISIONTRIANGLE_H__

#include "Vector.h"
#include "Color.h"
#include "RigidBodyState.h"
#include "SoftBodyState.h"
#include "AABB.h"
#include <memory>

namespace pba {

	class CollisionTriangleRaw
	{
	public:

		CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2)//construction method /hanshu function
		{
			P0 = p0;
			P1 = p1;
			P2 = p2;

			e1 = P1 - P0;
			e2 = P2 - P0;
			normal = e1 ^ e2;
			normal.normalize();
			//double det;

			Color color = Color(0.0,0.0,0.0,0.0);
			Color hitcolor = Color(0.2, 0.2, 0.2, 0.5);
			Color litcolor = Color(0.6, 0.6, 0.6, 0.6);

			/*hitcolor.red = 0.2;
			hitcolor.green = 0.2; 
			hitcolor.blue = 0.2;
			hitcolor.alpha = 0.5;
			litcolor.red = 0.6;
			litcolor.green = 0.6;
			litcolor.blue = 0.6;
			litcolor.alpha = 0.6;*/

			//Color hitcolor = (0.2,0.2,0.2,0.5);
			//Color litcolor = (0.6,0.6,0.6,0.6);
			//float decayrate = 0.5;
			decayrate = 0.5;
			//bool visible = true;
			visible = true;

			//AABB aa_bb;

			//bool is_in_triangle(const Vector& X);
		}
		~CollisionTriangleRaw() {}

		bool hit(const Vector& P, const Vector& V, const double tmax, double& t);

		bool hit(const RigidBodyState& s, const size_t i, const double tmax, double& t);

		bool hit(const SoftBodyState& s, const size_t i, const double tmax, double& t);

		bool hit(const Vector& P, const Vector& V, const double R, const double tmax, double& t);

		const Vector& N() const { return normal; }
		const Vector& vertex(int i) const
		{
			if (i == 2) { return P2; }
			else if (i == 1) { return P1; }
			return P0;
		}

		//const AABB& aabb() const { return aa_bb; }

		void set_visible() { visible = true; }
		void set_invisible() { visible = false; }
		bool visibility() const { return visible; }

		void set_color(const Color& c) { color = c; }
		const Color get_color() const { return color + hitcolor; }

		void set_hit_color(const Color& c) { hitcolor = c; }
		void set_lit_color(const Color& c) { litcolor = c; }


		void decay();

		void set_orientation(const Vector& P);

	private:

		Vector P0;
		Vector P1;
		Vector P2;
		Vector e1;
		Vector e2;
		Vector normal;
		double det;

		Color color;
		Color hitcolor;
		Color litcolor;
		float decayrate;
		bool visible;

		//AABB aa_bb;

		bool is_in_triangle(const Vector& X);
	};

	typedef std::shared_ptr<CollisionTriangleRaw> CollisionTriangle;

	CollisionTriangle makeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2);

}

#endif
