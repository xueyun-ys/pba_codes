//Various Dynamics Models Using Explicit Solvers


#include "ExplicitDynamics.h"
#include "CollisionHandler.h"
#include "Matrix.h"

using namespace pba;

AdvancePosition::AdvancePosition(DynamicalState& pq) :
	PQ( pq )
	{}

void AdvancePosition::init(){}

void AdvancePosition::solve(const double dt)
{
#pragma omp parallel for
	for (size_t i = 0; i < PQ->nb(); i++)
	{
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i)*dt);
	}
}

AdvanceVelocity::AdvanceVelocity(DynamicalState& pq, Force& f):
 	PQ(pq),
 	force(f)
	{}

void AdvanceVelocity::init(){}

void AdvanceVelocity::solve(const double dt)
{
	force->compute(PQ, dt);
#pragma omp parallel for
	for (size_t i = 0; i < PQ->nb(); i++)
	{
		PQ->set_vel(i, PQ->vel(i) + PQ->accel(i)*dt);
	}
}

pba::GISolver pba::CreateAdvancePosition(pba::DynamicalState&pq)
{
	return pba::GISolver(new pba::AdvancePosition(pq));
}

pba::GISolver pba::CreateAdvanceVelocity (pba::DynamicalState&pq, pba::Force& f)
{
	return pba::GISolver(new pba::AdvanceVelocity(pq , f));
}

AdvancePositionWithCollisions::AdvancePositionWithCollisions(pba::DynamicalState& pq, CollisionHandler& cs):
	PQ(pq),
	CS(cs)
{}

void AdvancePositionWithCollisions::init(){}
void AdvancePositionWithCollisions::solve(const double dt)//由leapfrog调用过来的
{
#pragma omp parallel for
	for (size_t i = 0; i < PQ->nb(); i++)
	{
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i)*dt);
	}
	CS.handle_collisions(dt, PQ);//................................
}

pba::GISolver pba::CreateAdvancePosition(DynamicalState& pq, CollisionHandler& cs)
{
	return pba::GISolver(new pba::AdvancePositionWithCollisions(pq, cs));//将变量存入advancedpositionwithcollisions中，返回
	//一个GISolver的子类,即创建了一个新的APWC并且调用了它的构造函数，而它又是子类可以算是GISolver
}

//AdvanceVelocityWithConstraint::AdvanceVelocityWithConstraint(DynamicalState& pq, Force& f):// , Constraint& c) :
//	PQ(pq),
//	force(f),
//	C(c)
//	{}

// void AdvanceVelocityWithConstraint::init(){}
// void AdvanceVelocityWithConstraint::solve(const double dt)
// {
// 	//force->compute(PQ, dt);
// 
// 	//graham-schmidt on force, and compute deviation force
// 	//right now, this is a one-particle set up
// #pragma omp parallel for
// 	for (size_t i = 0; i < PQ->nb(); i++)
// 	{
// 		Vector N = C->grad(PQ, i);
// 		double Nsq = N*N;
// 		if (Nsq > 0.0)
// 		{
// 			Vector F = PQ->accel(i)*PQ->mass(i);
// 			F -= ((F*N) / Nsq)*N;
// 
// 			Vector V = PQ->vel(i);
// 			double VN = V*N;
// 			Matrix M = C->gradgrad(PQ, i, i);
// 		}
// 	}
// }





