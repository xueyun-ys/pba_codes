#include "..\include\SPHState.h"

pba::SPHStateData::SPHStateData(/*const AABB& aabb,*/ const double h, const std::string& nam /*= "SPHDataNoName"*/) :
	DynamicalStateData(nam), /*OccupancyVolume(aabb, h),*/ radius(2 * h)
{
	create_attr("den", 0.0f);
}

pba::SPHStateData::~SPHStateData()
{
}

void pba::SPHStateData::set_radius(const float & v)
{
	radius = v;
}

const float pba::SPHStateData::weight(size_t p, const Vector & P) const//calculate the weight(w(h, r))
{
	Vector rab = P - pos(p);
	float h = radius / 2.0;
	float r = rab.magnitude();
	float temp;// = rab * rab * (-1.0) / (h * h);
	//temp = 1.0 / (h *h *h)
	float w = 1.0 / (h * h * h * PI);

	if ( r >= 0.0 && r <= h)
	{
		temp = 1.0 - 3.0*((r / h)*(r / h)) + 0.75 * (r / h)*(r / h)*(r / h);
		return w * temp;
	}

	else if (r >= h&& r <= radius)
	{
		temp = 2.0 - (r / h);
		temp = temp * temp * temp * 0.25;
		return w * temp;
	}

	else if (r > radius)
	{
		return 0.0;
	}

	else if (r < 0.0)
	{
		return 0.0;
	}
	return 0.0;
}

const pba::Vector pba::SPHStateData::grad_weight(size_t p, const Vector & P) const
{
	Vector rab = P - pos(p);
	float h = radius / 2.0;
	float r = rab.magnitude();
	float w = 1.0 / (h * h * h * h * PI);
	float temp = 0.0;
	Vector xab;
	/*if (pos(p) == P)
		xab = Vector(0.001,0.001,0.001);
	else*/
	xab = (pos(p) - P) / ((pos(p) - P).magnitude());
	if (pos(p) == P)
		xab = Vector(.0, .0, .0);

	Vector wab = rab / r;
	if (r >= 0&& h >= r)
	{
		temp = -6.0 * (r / h) + 2.25*(r / h)*(r / h);
		temp *= w;
		Vector test = temp * xab;
		return temp * xab;
	}
	else if (r > h&& r <= 2.0*h)
	{
		temp = -0.75 * (2.0 - r / h) * (2.0 - r / h);
		Vector test = temp * w * xab;
		return temp * w * xab;
	}
	else if (r >= radius)
	{
		Vector test = xab * 0.0;
		return xab * 0.0;
	}
	else if(r < 0.0)
		return xab * 0.0;
	return Vector();
}

void pba::SPHStateData::compute_density()
{
	float Pa;
	for (int j = 0; j < nb(); j++)
	{
		Pa = 0.0;
		for (int i = 0; i < nb(); i++)
		{
			if (i != j)
			{
				Pa = Pa + mass(i) * weight(j, pos(i));
			}
		}
		set_attr("den", j, Pa);//..................map? density for particle a.
	}
	
}

pba::SPHState pba::CreateSPH(/*const AABB & bounds,*/ const double h, const std::string & nam)
{
	return SPHState(new SPHStateData(/*bounds,*/ h, nam));
}

//pba::SPHState pba::copy(const SPHState d)
//{
//	SPHState S = CreateSPH(d->bounds, d->get_radius(), d->Name());//d. ???
//	//radius = d.radius;
//	return S;
//}
