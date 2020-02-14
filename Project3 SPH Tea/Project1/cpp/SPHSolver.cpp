#include "..\include\SPHSolver.h"
#include "ForceLibrary.h"

//===============================================================
//AdvanceSPHPosition
pba::AdvanceSPHPosition::AdvanceSPHPosition(SPHState & pq)
{
	PQ = pq;
}

void pba::AdvanceSPHPosition::init()
{
}

void pba::AdvanceSPHPosition::solve(const double dt)
{
	/*for (int i = 0; i < pq->nb(); i++)
	{
	pq->set_pos(i, );
	}*/
}

pba::GISolver pba::CreateAdvancePosition(SPHState & pq)
{
	return GISolver(new AdvanceSPHPosition(pq));//new的话返回指针,指针存储了地址
}


//======================================================================two solvers for seperately updating v && p
//AdvanceSPHVelocity
pba::GISolver pba::CreateAdvanceVelocity(SPHState & pq, Force & f, float vel_clamp, float accel_clamp)
{
	return GISolver(new AdvanceSPHVelocity(pq, f, vel_clamp, accel_clamp));
}

pba::AdvanceSPHVelocity::AdvanceSPHVelocity(SPHState & pq, Force & f, float vclamp, float aclamp)
{
	PQ = pq;
	force = f;
	velocity_clamp = vclamp;
	acceleration_clamp = aclamp;
}

void pba::AdvanceSPHVelocity::init()
{
}

void pba::AdvanceSPHVelocity::solve(const double dt)
{

	// update accel
	for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_accel(i, Vector(0.0, 0.0, 0.0));
	}

	std::shared_ptr<DynamicalStateData> pq = std::dynamic_pointer_cast<DynamicalStateData>(PQ);//类型转换
	std::shared_ptr<AccumulatingForce> f = std::dynamic_pointer_cast<AccumulatingForce>(force);

	// External Force
	force->compute(pq, dt);
	// Pressure & Viscosity Force
	force->compute(PQ, dt);//SPHStateData type para,meter参数?

	// update velocity
	for (int i = 0; i < PQ->nb(); i++)
	{
		if (PQ->accel(i).magnitude() > acceleration_clamp)
			PQ->set_accel(i, PQ->accel(i).unitvector() * acceleration_clamp);

		Vector _v = PQ->vel(i) + PQ->accel(i) * dt;
		if (_v.magnitude() > velocity_clamp)
			PQ->set_vel(i, _v.unitvector() * velocity_clamp);
		else
			PQ->set_vel(i, _v);
		//PQ->set_vel(i, PQ->vel(i) + PQ->accel(i) * dt);
	}
}

//=============================================================================
//AdvanceSPHPositionWithCollisions
pba::AdvanceSPHPositionWithCollisions::AdvanceSPHPositionWithCollisions(SPHState & pq, ElasticCollisionHandler & coll)//&符号的两重含义[引用指针地址区别]
{
	PQ = pq;
	CS = &coll;
}
//pba::AdvanceSPHPositionWithCollisions::AdvanceSPHPositionWithCollisions(SPHState& pq, ElasticCollisionHandler& coll) :
//	PQ(pq), CS(&coll)
//{}

void pba::AdvanceSPHPositionWithCollisions::init()
{

}

void pba::AdvanceSPHPositionWithCollisions::solve(const double dt)
{
	// update position
	for (int i = 0; i < PQ->nb(); i++)
	{
		//assert(mag_pos > (-1000.0));
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}

	// handle collisions
	DynamicalState ds = std::dynamic_pointer_cast<DynamicalStateData>(PQ);//内存一致
	CS->handle_collisions(dt, ds);

	// compute density
	PQ->compute_density();
}

pba::GISolver pba::CreateAdvancePosition(SPHState & pq, ElasticCollisionHandler & cs)
{
	return GISolver(new AdvanceSPHPositionWithCollisions(pq, cs));
}