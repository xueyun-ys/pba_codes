#include "CollisionTriangle.h"
//#include "Vector.h"  在上面那个头文件中已经引用

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
		inface = is_in_triangle(hitpos);
		if (!inface)
		{
			hit_triangle = false;
		}
	}
	
	return hit_triangle;
}

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


pba::CollisionTriangle pba::makeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2)
{
	return CollisionTriangle(new CollisionTriangleRaw(p0, p1, p2));
}