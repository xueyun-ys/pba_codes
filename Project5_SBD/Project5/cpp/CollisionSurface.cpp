#include "CollisionSurface.h"

pba::CollisionSurfaceRaw::CollisionSurfaceRaw()
{
	coeff_of_restitution = 1.0;
	coeff_of_sticky = 1.0;//double coeff_of_st.. &double?
	wireframe = false;
}


pba::CollisionSurface makeCollisionSurface()
{
	return pba::CollisionSurface(new pba::CollisionSurfaceRaw());
}


void pba::CollisionSurfaceRaw::addTriangle(const CollisionTriangle& t)
{
	tri_elements.push_back(t);//collisionsurface对象里的tri_elements
}

void pba::CollisionSurfaceRaw::addSphere(const CollisionSphere& t)
{
	sphere_elements.push_back(t);
}
//======================================================General  hits  ===========================================================================
bool pba::CollisionSurfaceRaw::hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const
{
	double tb = 0.0;
	t.status = false;
	for (size_t i = 0; i < tri_elements.size(); i++)//空间中的三角形的数据存储在triangle中，surface中存储了包含所有的triangle对象的数组
	{
		if (tri_elements[i]->hit(P, V, tmax, t.t))
		{
			t.status = true;
			t.hit_sph = true;
			t.tri = tri_elements[i];
		}
		if (t.t > tb && t.status)//再次更新，找到了碰撞更大的时间点t.t
		{
			tb = t.t;
			t.hit_index = i;
			t.tri = tri_elements[i];
		}

		/*double t;
		CollisionTriangle tri;
		CollisionSphere sph;
		bool hit_tri;
		bool hit_sph;
		size_t hit_index;*/
	}
	t.t = tb;//碰撞检测时间

	return t.status;
	//bool hitsurf = pba::CollisionSurfaceRaw::hit(P,  V,  tmax, t.t) //自己最开始的想法
}
//=================================================================General========================================================================


bool pba::CollisionSurfaceRaw::hit(const Vector& P, const Vector& V, const double R, const double tmax, CollisionData& t) const
{
	//what's R？
	return false;
}

//=================================================================RBD========================================================================
//cgwpc
bool pba::CollisionSurfaceRaw::hit(const RigidBodyState& s, const size_t _i, const double tmax, CollisionData& t) const
{
	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int j = 0; j < tri_elements.size(); j++)
	{
		if (tri_elements[j]->hit(s, _i, tmax, tc))//collision triangle
		{
			// find the largest backwards T (tc)
			if (isFirst) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
			}
		}
	}

	return t.status;
}
//=================================================================RBD========================================================================


//==================================================================SBD=========================================================================
bool pba::CollisionSurfaceRaw::hit(const SoftBodyState & s, const size_t i, const double tmax, CollisionData & t) const
{
	//double tb = 0.0;
	//t.status = false;
	//for (size_t l = 0; l < tri_elements.size(); l++)//空间中的三角形的数据存储在triangle中，surface中存储了包含所有的triangle对象的数组
	//{
	//	if (tri_elements[l]->hit(s, i, tmax, t.t))
	//	{
	//		t.status = true;
	//		t.hit_sph = true;
	//		t.tri = tri_elements[l];
	//	}
	//	if (t.t > tb && t.status)//再次更新，找到了碰撞更大的时间点t.t
	//	{
	//		tb = t.t;
	//		t.hit_index = l;
	//		t.tri = tri_elements[l];
	//	}

	//}
	//t.t = tb;//碰撞检测时间

	//if (t.status)
	//{
	//	//system("pause");
	//}
	//return t.status;
	//===================================================collision ofcg=========================================================
	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int j = 0; j < tri_elements.size(); j++)
	{
		if (tri_elements[j]->hit(s, i, tmax, tc))
		{
			// find the largest backwards T (tc)
			if (isFirst) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
			}
		}
	}

	return t.status;
	//===================================================collision ofcg=========================================================

}


//====================================================SBD=======================================================================================

pba::CollisionSurface pba::makeCollisionSurface()
{
		return CollisionSurface(new(CollisionSurfaceRaw));
}
