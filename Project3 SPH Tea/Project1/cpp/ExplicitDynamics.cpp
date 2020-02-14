//Various Dynamics Models Using Explicit Solvers


#include "ExplicitDynamics.h"
#include "CollisionHandler.h"
#include "Matrix.h"
#include "ConstraintLibrary.h"
#include <memory>


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


//========================================================================================================================
AdvancePositionWithCollisions::AdvancePositionWithCollisions(pba::DynamicalState& pq, CollisionHandler& cs):
	PQ(pq),
	CS(cs)
{}

void AdvancePositionWithCollisions::init(){}
void AdvancePositionWithCollisions::solve(const double dt)//��leapfrog���ù�����
{
#pragma omp parallel for
	for (size_t i = 0; i < PQ->nb(); i++)
	{
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i)*dt);
	}
	CS.handle_collisions(dt, PQ);//................................��ʼ�����ײ
}

pba::GISolver pba::CreateAdvancePosition(DynamicalState& pq, CollisionHandler& cs)
{
	return pba::GISolver(new pba::AdvancePositionWithCollisions(pq, cs));//����������advancedpositionwithcollisions�У�����
	//һ��GISolver������,��������һ���µ�APWC���ҵ��������Ĺ��캯�����������������������GISolver
}


//=============================================================================================================================



pba::AdvanceVelocityWithConstraint::AdvanceVelocityWithConstraint(DynamicalState& pq, Force& f, MultiConstraint& c) :
	PQ(pq),
	force(f),
	C(c)
	{}

void pba::AdvanceVelocityWithConstraint::init()
{

}

void pba::AdvanceVelocityWithConstraint::solve(const double dt)
{
	//���㵱ǰ����=����/flocking
	force->compute(PQ, dt);//���ݳ�ʼ�������ȷ��force��ʲô���ͣ�ִ�ж�Ӧ��force��compute
	
	//����Sv
	//F=?�����ô��con
	Vector vs;
	for (size_t j = 0; j < PQ->nb(); j++)
	{
		//Constraint constraint = C->get_constraint(j);
		//std::shared_ptr<ParticleOnSphereConstraint> con = dynamic_pointer_cast<ParticleOnSphereConstraint>(constraint);
		//double R = con->get_radius();
		//Vector P = con->get_Spherecenter();
		double R = 1.0;
		Vector P = Vector(.0, .0, .0);


		double Ks = C->get_constraint(0)->get_Ks();
		double Kf = C->get_constraint(0)->get_Kf();
		double cx = (PQ->pos(j) - P) * (PQ->pos(j) - P) - R* R;
		Vector forceF = PQ->mass(j) * PQ->accel(j);
		Vector nn = C->grad(PQ, 0, j);
		Matrix M = C->gradgrad(PQ, 0, j, j);
		double Nsq = nn * nn;//n��ƽ��
		if (Nsq > 0.0)
		{
			PQ->set_vel(j, PQ->vel(j) + dt * (forceF / PQ->mass(j) - (nn * forceF / PQ->mass(j)) * (nn / Nsq) - PQ->vel(j) * M * PQ->vel(j) / Nsq * nn));
			PQ->set_vel(j, PQ->vel(j) + dt * (nn/Nsq) * (Ks * cx + Kf * PQ->vel(j)*nn) );//relaxation
			PQ->set_vel(j, PQ->vel(j) - nn * PQ->vel(j) * nn / Nsq);
		}

	}




 
	///graham-schmidt on force, and compute deviation force
	///right now, this is a one-particle set up
	//#pragma omp parallel for
	//for (size_t i = 0; i < PQ->nb(); i++)
	//{
 //		//Vector N = C->grad(PQ, 0, i);
 //		//double Nsq = N*N;
 //		if (Nsq > 0.0)
 //		{
 //			Vector F = PQ->accel(i)*PQ->mass(i);
 //			F -= ((F*N) / Nsq)*N;
 //
 //			Vector V = PQ->vel(i);
 //			double VN = V*N;
 //			Matrix M = C->gradgrad(PQ, 0, i, i);
 //		}
	//}
}





pba::GISolver pba::CreateAdvanceVelocity(DynamicalState& pq, Force& f, MultiConstraint& c)
{
	return pba::GISolver(new pba::AdvanceVelocityWithConstraint(pq, f, c));//&pq����??????????
}


//==============
pba::AdvancePositionWithConstraint::AdvancePositionWithConstraint(DynamicalState& pq, MultiConstraint& c) :
	PQ(pq),
	C(c)
{}

void pba::AdvancePositionWithConstraint::init()
{

}

void pba::AdvancePositionWithConstraint::solve(const double dt)
{
	///////////////////////////////////////////////////////////////;
	for (size_t j = 0; j < PQ->nb(); j++)
	{
		Vector x0 = PQ->pos(j);
		PQ->set_pos(j, PQ->vel(j) * dt + PQ->pos(j));
		C->solve(PQ, 0.05, 100);//����C(X)�Ĵ�С.........................�Ĳ���
		PQ->set_vel(j, (PQ->pos(j) - x0) / dt);

	}
}


pba::GISolver pba::CreateAdvancePosition(DynamicalState& pq, MultiConstraint& cs)
{
	return pba::GISolver(new pba::AdvancePositionWithConstraint(pq,  cs));
}



//=====================


//pba::GISolver pba::CreateAdvanceVelocity(DynamicalState& pq, Force& f, MultiConstraint& c)
//{
//	return pba::GISolver(new pba::AdvanceVelocityWithoutConstraint(pq, f, c));
//}


pba::AdvanceVelocityWithoutConstraint::AdvanceVelocityWithoutConstraint(DynamicalState& pq, Force& f, MultiConstraint& c) :
	PQ(pq),
	force(f),
	C(c)
{}

pba::GISolver pba::CreateAdvanceVelocity2(DynamicalState& pq, Force& f, MultiConstraint& c)
{
	return pba::GISolver(new pba::AdvanceVelocityWithoutConstraint(pq,f, c));
}


void pba::AdvanceVelocityWithoutConstraint::solve(const double dt)
{
	//������
	//���㵱ǰ����=����/flocking
	force->compute(PQ, dt);//���ݳ�ʼ�������ȷ��force��ʲô���ͣ�ִ�ж�Ӧ��force��compute
	Vector vs;
	for (size_t j = 0; j < PQ->nb(); j++)
	{
		double R = 1.0;
		Vector P = Vector(.0, .0, .0);


		double Ks = C->get_constraint(0)->get_Ks();
		double Kf = C->get_constraint(0)->get_Kf();
		double cx = (PQ->pos(j) - P) * (PQ->pos(j) - P) - R* R;
		Vector forceF = PQ->mass(j) * PQ->accel(j);
		Vector nn = C->grad(PQ, 0, j);
		Matrix M = C->gradgrad(PQ, 0, j, j);
		double Nsq = nn * nn;//n��ƽ��
		if (Nsq > 0.0)
		{
			PQ->set_vel(j, PQ->vel(j) + dt * (forceF / PQ->mass(j) - (nn * forceF / PQ->mass(j)) * (nn / Nsq) - PQ->vel(j) * M * PQ->vel(j) / Nsq * nn));
			//PQ->set_vel(j, PQ->vel(j) + dt * (nn / Nsq) * (Ks * cx + Kf * PQ->vel(j)*nn));//relaxation
			PQ->set_vel(j, PQ->vel(j) - nn * PQ->vel(j) * nn / Nsq);
		}

	}
}
