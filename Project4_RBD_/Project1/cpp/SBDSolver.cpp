#include "..\include\SBDSolver.h"

pba::AdvanceSoftBodyPositionWithCollisions::AdvanceSoftBodyPositionWithCollisions(SoftBodyState & pq, ElasticSBDCollisionHandler & coll):PQ(pq), CS(coll)
{}

void pba::AdvanceSoftBodyPositionWithCollisions::solve(const double dt)
{
	// update position
	for (int i = 0; i < PQ->nb(); i++)
	{
		//assert(mag_pos > (-1000.0));
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}

	// handle collisions
	CS.handle_collisions(dt, PQ);

	

}

pba::GISolver pba::CreateAdvanceRotation(SoftBodyState & pq)
{
	return GISolver(new AdvanceSoftBodyPosition(pq));
}
//====================================================================================================
pba::GISolver pba::CreateAdvanceAngularVelocity(SoftBodyState & pq, Force & f)
{
	return GISolver(new AdvanceSoftBodyVelocity(pq, f));
}

pba::GISolver pba::CreateAdvanceRotation(SoftBodyState & pq, ElasticSBDCollisionHandler & cs)
{
	return GISolver(new AdvanceSoftBodyPositionWithCollisions(pq, cs));
}

pba::AdvanceSoftBodyVelocity::AdvanceSoftBodyVelocity(SoftBodyState & pq, Force & f):PQ(pq), force(f)
{}

void pba::AdvanceSoftBodyVelocity::solve(const double dt)
{
	// update accel
	for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_accel(i, Vector(0.0, 0.0, 0.0));
	}

	// External Force
	//force->compute(pq, dt);
	// Pressure & Viscosity Force
	force->compute(PQ, dt);
	//for (int i = 0; i < PQ->nb(); i++)
	//	PQ->set_accel(i, Vector(0.0, -1.0, 0.0));

	// update velocity
	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector _v = PQ->vel(i) + PQ->accel(i) * dt;
		PQ->set_vel(i, _v);
	}
}
