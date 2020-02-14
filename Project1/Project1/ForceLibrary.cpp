#include "ForceLibrary.h"

void pba::SimpleGravityForce::compute(pba::DynamicalState& pq, const double dt)
{
	for (int i =0; i<pq->nb(); i++)
	{
		pba::Vector acc = G / pq->mass(i);
		pq->set_accel(i, acc);
	}
	
}

pba::Force pba::CreateSimpleGravityForce(const Vector& G)
{

	return Force(new SimpleGravityForce(G));
}

