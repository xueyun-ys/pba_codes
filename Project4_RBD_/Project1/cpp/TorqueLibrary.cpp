#include "TorqueLibrary.h"

void pba::TorqueFromForce::compute(RigidBodyState & s, const double dt)
{
	Vector temp;
	Matrix M;
	for (int i = 0; i < s->nb(); i++)
	{
		//outer_product(,,M);
		temp += (s->pos(i) - s->center_of_mass) ^ (s->mass(i) * s->accel(i));//又算了一次，浪费 outer product != exterior product
	}
	s->set_attr("torque", 0, temp);
}

pba::Torque pba::CreateTorqueFromForce(Force & f)
{
	return Torque(new TorqueFromForce(f));
}
