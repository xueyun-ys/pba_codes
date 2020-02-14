#include "ForceLibrary.h"
#include "PbaUtils.h"
#include "Mymethods.h"
#include "SPHForce.h"

//������������������ٶ�
void pba::SimpleGravityForce::compute(pba::DynamicalState& pq, const double dt)
{
	for (int i =0; i<pq->nb(); i++)
	{
		pba::Vector acc = G / pq->mass(i);
		pq->set_accel(i, pq->accel(i) + acc);
	}
	
}

pba::Force pba::CreateSimpleGravityForce(const Vector& G)
{

	return Force(new SimpleGravityForce(G));
}

//===============================================AccumulatingForce for SPHState& DynamicalState(��ͬ��compute()��Ҫ��ͬ�Ĳ���)
void pba::AccumulatingForce::compute(SPHState & s, const double dt)
{
	/*for (int i = 0; i < forces.size(); i++)
	{
		forces[i]->compute(s, dt);
	}*/
	for (auto f : forces)
	{
		std::shared_ptr<SPHForce> SPHF = std::dynamic_pointer_cast<SPHForce>(f);
		if (!SPHF)
			continue;
		SPHF->compute(s, dt);
	}
}

//==========================================================================================================================
//����flocking�ĸ������ٶȼ���
void pba::AccumulatingForce::compute(pba::DynamicalState& pq, const double dt)
{
	/*for (auto f : forces)
		f->compute(pq, dt);*/
	for (auto f : forces)
	{
		std::shared_ptr<SPHForce> SPHF = std::dynamic_pointer_cast<SPHForce>(f);
		if (SPHF)
			continue;
		f->compute(pq, dt);
	}

}
//=============================================
void pba::AccumulatingForce::add(Force& f)
{
	forces.push_back(f);
}

pba::Force pba::CreateAccumulatingForce()
{
	return pba::Force(new pba::AccumulatingForce);
}
//==========================================================================================

void pba::AccumulatingBoidCollisionAvoidanceForce::compute(pba::DynamicalState& pq, const double dt)
{

}

pba::Force pba::CreateAccumulatingBoidCollisionAvoidanceForce(const double G)
{
	return pba::Force(new pba::AccumulatingBoidCollisionAvoidanceForce(G));
}
//=========================================BoidForce=================================================
void pba::AccumulatingBoidForce::compute(pba::DynamicalState& pq, const double dt)
{
	double t1 = std::cos(dfov  * 3.14159265 / 180.0 /2.0);//���
	double t2 = std::cos(fov * 3.14159265 / 180.0 /2.0);

	for (int i = 0; i < pq->nb(); i++)
	{
		double kr = 0, kf = 0;
		Vector a_avoid, a_velMat, a_center;
		double _amax = amax;

		for (int j = 0; j < pq->nb(); j++)
		{
			if (j == i)
				continue;
			//�õ����ֵܵ�λ�ú��ٶ�������,��������j��i������
			Vector xa = pq->pos(i);
			Vector xb = pq->pos(j);
			Vector va = pq->vel(i);
			Vector vb = pq->vel(j);

			//compute Influence Range
			double r = (xa - xb).magnitude();

			if (r < range)
				kr = 1;
			else if (r > range && r < range_ramp)
				kr = (range_ramp - r) / (range_ramp - range);
			else if (r > range_ramp)
				kr = 0;

			//compute Influence FOV
			//double t = ((xb - xa) * va) / ((xb - xa).magnitude() * va.magnitude());
			double cosab = (xb - xa).unitvector() * va.unitvector();

			if (cosab > t1)
				kf = 1;
			else if (cosab > t2 && cosab < t1)
				kf = (t2 - cosab) / (t2 - t1);
			else if (cosab < t2)
				kf = 0;

			// Avoidance���ٶ�������һ��
			a_avoid += (A * (xa - xb).unitvector() * (1 / (xa - xb).magnitude()) * kr * kf);

			// Velocity Matching���ٶ�������һ��
			a_velMat += (V * (vb - va) * kr * kf);

			// Centering���ٶ�������һ��
			a_center += (C * (xb - xa) * kr * kf);

		}// particle b

		 // Acceleration Prioritization
		//������������ٶȵ�ֵ�Ƿ񳬱�
		double a_len = a_avoid.magnitude();
		if (a_len > _amax) 
		{
			a_avoid = _amax * a_avoid.unitvector();
			a_velMat = Vector(0.0, 0.0, 0.0);
			a_center = Vector(0.0, 0.0, 0.0);
		}
		else 
		{
			_amax = _amax - a_len;
			a_len = a_velMat.magnitude();

			if (a_len > _amax) 
			{
				a_velMat = _amax * a_velMat.unitvector();
				a_center = Vector(0.0, 0.0, 0.0);
			}
			else 
			{
				_amax = _amax - a_len;
				a_len = a_center.magnitude();

				if (a_len > _amax) 
				{
					a_center = _amax * a_center.unitvector();
				}
			}
		}
		//�����������ܵļ��ٶ�
		Vector _aTotal = a_avoid + a_velMat + a_center;

		// if the boid is the lead boid
		if (i == leadBoidID) {
			Vector dir(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);//0~1ת��Ϊ-0.5~0.5
			_aTotal = dir.unitvector() * _aTotal.magnitude();//����ı����ķ���
		}

		pq->set_accel(i, _aTotal);

	} // particle a
}

pba::Force pba::CreateAccumulatingBoidForce(const double A, const double V, const double C, const double Max, const double range, const double range_ramp)
{
	return pba::Force(new pba::AccumulatingBoidForce(A, V, C, Max, range, range_ramp));
}
//=============================================BoidForce=============================================