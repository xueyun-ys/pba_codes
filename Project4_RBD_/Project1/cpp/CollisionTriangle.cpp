#include "CollisionTriangle.h"
//#include "Vector.h"  在上面那个头文件中已经引用

//whether in the triangle or not
bool pba::CollisionTriangleRaw::is_in_triangle(const Vector& X)
{
	bool in_triangle = false;
	Vector temp;
	temp = e2^e1;
	double u = (e2^e1)*(e2 ^ (X - P0)) / ((e2^e1).magnitude() * (e2^e1).magnitude());
	double v = (e1^e2)*(e1 ^ (X - P0)) / ((e1^e2).magnitude() * (e1^e2).magnitude());
	if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && (u + v) >= 0 && (u + v) <= 1)
	{
		in_triangle = true;
	}
	return in_triangle;
}

//======================================================hit triangle=========================================================

//=========================== general =========================================
bool pba::CollisionTriangleRaw::hit(const Vector& P, const Vector& V, const double tmax, double& t)
{
	bool hit_triangle;
	//t = tmax;
	Vector temp = P - V*tmax;
	temp = temp - P0;
	double t1 = temp * normal;
	temp = P - P0;
	double t2 = temp * normal;
	hit_triangle = t1*t2 < 0 ? true : false;

	if (hit_triangle)
	{
		//return false;
	}

	double tt = normal*(P-P0)/(normal * V);
	t = tt;


	double hittime = normal * (P - P0) / (normal*V);
	if (hittime < 0.000001 && hit_triangle)
	{
		hit_triangle = false;	
	}

	//return is_in_triangle()
	bool inface;
	Vector hitpos = P - (V * t);
	if (hit_triangle)
	{
		//hit_triangle = is_in_triangle
		inface = is_in_triangle(hitpos);
		if (!inface)
		{
			hit_triangle = false;
		}
	}
	
	return hit_triangle;
}
//=========================== general =========================================


//=========================== rigid body =========================================
//cgwxp
bool pba::CollisionTriangleRaw::hit(const RigidBodyState& s, const size_t i, const double tmax, double& t)
{
	Matrix u = rotation(s->angular_velocity, s->angular_velocity.magnitude() * tmax) * s->angular_rotation;
	Vector x0 = s->center_of_mass - s->linear_velocity * tmax + u * s->get_vector_attr("p", i);
	Vector x1 = s->pos(i);

	// Detect a collision has happened
	double f0 = (x0 - P0) * normal;
	double f1 = (x1 - P0) * normal;
	if ((f0 * f1) > 0)
		return false;

	// Compute where and when collision takes place
	double t0 = 0, t1 = tmax;
	Vector xc;
	double fc;
	while (true)
	{
		t = 0.5 * (t0 + t1);

		u = rotation(s->angular_velocity, s->angular_velocity.magnitude() * t) * s->angular_rotation;
		xc = s->center_of_mass - s->linear_velocity * t + u * s->get_vector_attr("p", i);
		fc = (xc - P0) * normal;

		if (fc == 0) {
			t = (t0 + t1) * 0.5;
			break;
		}

		if (f0 * fc < 0) {
			f1 = fc;
			t0 = t;
		}
		else if (f0 * fc > 0) {
			f0 = fc;
			t1 = t;
		}

		if (abs(abs(t0 - t1) / tmax) < 0.0001)
			break;
	}

	u = rotation(s->angular_velocity, s->angular_velocity.magnitude() * t)* s->angular_rotation;
	xc = s->center_of_mass - s->linear_velocity*t + u * s->get_vector_attr("p", i);

	// not a collision
	if ((t * tmax < 0) || (((tmax - t) / tmax) < 1e-6))
		return false;

	return is_in_triangle(xc);
}

//=========================== rigid body =========================================

//=========================== soft body =========================================
bool pba::CollisionTriangleRaw::hit(const SoftBodyState & s, const size_t i, const double tmax, double & t)
{
	bool hit_triangle;
	Vector temp = s->pos(i) - s->vel(i) * tmax;
	temp = temp - P0;
	double t1 = temp * normal;
	temp = s->pos(i) - P0;
	double t2 = temp * normal;
	hit_triangle = t1*t2 < 0 ? true : false;

	if (hit_triangle)
	{
		//return false;
	}

	double tt = normal*(s->pos(i) - P0) / (normal * s->vel(i));
	t = tt;


	double hittime = normal * (s->pos(i) - P0) / (normal*s->vel(i));
	if (hittime < 0.000001 && hit_triangle)
	{
		hit_triangle = false;
	}

	bool inface;
	Vector hitpos = s->pos(i) - (s->vel(i) * t);
	if (hit_triangle)
	{
		inface = is_in_triangle(hitpos);
		if (!inface)
		{
			hit_triangle = false;
		}
	}

	return hit_triangle;
}
//=========================== soft body =========================================

//======================================================hit triangle=========================================================
pba::CollisionTriangle pba::makeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2)
{
	return CollisionTriangle(new CollisionTriangleRaw(p0, p1, p2));
}