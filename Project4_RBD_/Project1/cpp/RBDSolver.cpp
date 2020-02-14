#include "RBDSolver.h"
#include "RigidBodyState.h"
#include "Torque.h"
#include "TorqueLibrary.h"
#include "GISolver.h"
#include "CollisionHandler.h"
#include "LinearAlgebra.h"


pba::AdvanceRotationAndCOM::AdvanceRotationAndCOM(RigidBodyState& pq) :
	PQ(pq)
{}

void pba::AdvanceRotationAndCOM::init()
{

}

void pba::AdvanceRotationAndCOM::solve(const double dt)//返回值，类名，方法;为何是指针呢pq
{
	//update Sv, Sx for particles
	/*for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_vel(i, PQ->vel(i) + PQ->accel(i));
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i));
	}*/
	

	//update Sx, SR
	/*PQ->center_of_mass += dt * PQ->linear_velocity;
	PQ->angular_rotation = rotation(PQ->angular_velocity.unitvector(), PQ->angular_velocity.magnitude()*-1.0 *dt) * PQ->angular_rotation;*/
}

//=====================================================================================================
void pba::AdvanceAngularVelocityAndVelocity::init()
{
}

void pba::AdvanceAngularVelocityAndVelocity::solve(const double dt)//solverb for leap frog
{
	//pba::AdvanceRotationAndCOM::solve(dt);
	//update Sx, SR
	/*PQ->center_of_mass += dt * PQ->linear_velocity;
	PQ->angular_rotation = rotation(PQ->angular_velocity.unitvector(), PQ->angular_velocity.magnitude()*-1.0 *dt) * PQ->angular_rotation;*/

	//update Sv
	Vector Fcm;
	for (int i = 0; i < PQ->nb(); i++)
	{
		Fcm += PQ->mass(i) * PQ->accel(i);
	}
	PQ->linear_velocity = PQ->linear_velocity + Fcm * dt / PQ->totalmass();

	//update Sw
	//compute I
	PQ->recompute_MOI();//方法也要用箭头吗
	//compute torque
	tau->compute(PQ ,dt);
	//update w
	PQ->angular_velocity += PQ->inverse_moi() * PQ->get_vector_attr("torque", 0) * dt;//tau->compute(PQ, dt)* dt;
	//update Velocity/Sv,Sp
	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector Ua = PQ->angular_velocity ^ PQ->get_vector_attr("r", i);
		Vector vel = PQ->linear_velocity + Ua;
		PQ->set_vel(i, vel);
	}
}

pba::AdvanceAngularVelocityAndVelocity::AdvanceAngularVelocityAndVelocity(RigidBodyState& pq, Force& f) :
	PQ(pq)
{
	tau = CreateTorqueFromForce(f);//pba::?
}

//=====================================================================================================
pba::AdvanceRotationWithCollisions::AdvanceRotationWithCollisions(RigidBodyState & pq, ElasticRBDCollisionHandler & coll)://solver a for leapfrog
	CS(coll),PQ(pq)
{
	//PQ = pq;//......因为没有&所以报错？老师忘记加了?
	//CS = coll;
}

void pba::AdvanceRotationWithCollisions::init()
{
}

void pba::AdvanceRotationWithCollisions::solve(const double dt)
{
	//update Sv, Sx for particles
	/*for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_vel(i, PQ->vel(i) + PQ->accel(i));
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i));
	}*/

	//update Sx, SR
	PQ->center_of_mass += dt * PQ->linear_velocity;
	//PQ->angular_rotation = rotation(PQ->angular_velocity.unitvector(), PQ->angular_velocity.magnitude()*-1.0 *dt) * PQ->angular_rotation;
	double value = PQ->angular_velocity.magnitude();
	PQ->angular_rotation = rotation(PQ->angular_velocity.unitvector(), -value * dt)* PQ->angular_rotation;

	// Update Position/Sp
	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector pos = PQ->center_of_mass + PQ->angular_rotation * PQ->get_vector_attr("p", i);
		PQ->set_pos(i, pos);
		PQ->set_attr("r", i, (PQ->angular_rotation * PQ->get_vector_attr("p", i)));
	}

	CS.handle_collisions(dt, PQ); 
}
//=======================================================================================================




//============================================create============================================================
//out of classes
pba::GISolver pba::CreateAdvanceRotation(RigidBodyState & pq)
{
	return GISolver(new AdvanceRotationAndCOM(pq));
}

pba::GISolver pba::CreateAdvanceAngularVelocity(RigidBodyState & pq, Force & f)
{
	return GISolver(new AdvanceAngularVelocityAndVelocity(pq, f));
}

pba::GISolver pba::CreateAdvanceRotation(RigidBodyState & pq, ElasticRBDCollisionHandler & cs)
{
	return GISolver(new AdvanceRotationWithCollisions(pq, cs));
}
//============================================create============================================================