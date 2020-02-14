#include "ForceLibrary.h"
#include "PbaUtils.h"
#include "Mymethods.h"
#include "SPHForce.h"

//关于最基本的重力加速度
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

//===============================================AccumulatingForce for SPHState& DynamicalState(不同的compute()需要不同的参数)
void pba::AccumulatingForce::compute(SPHState & s, const double dt)
{
	/*for (int i = 0; i < forces.size(); i++)
	{
		forces[i]->compute(s, dt);
	}*/
	for (auto f : forces)
	{
		/*std::shared_ptr<SPHForce> SPHF = std::dynamic_pointer_cast<SPHForce>(f);
		if (!SPHF)
			continue;
		SPHF->compute(s, dt);*/
		f->compute(s, dt);
	}
}

//关于flocking的各个加速度计算
void pba::AccumulatingForce::compute(pba::DynamicalState& pq, const double dt)
{
	/*for (auto f : forces)
		f->compute(pq, dt);*/
	for (auto f : forces)
	{
		//std::shared_ptr<SPHForce> SPHF = std::dynamic_pointer_cast<SPHForce>(f);
		//if (SPHF)
		//	continue;
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
//==========================================================================================
void pba::AccumulatingBoidForce::compute(pba::DynamicalState& pq, const double dt)
{
	double t1 = std::cos(dfov  * 3.14159265 / 180.0 /2.0);//半角
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
			//得到两兄弟的位置和速度来计算,计算所有j到i的作用
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

			// Avoidance加速度作用在一起
			a_avoid += (A * (xa - xb).unitvector() * (1 / (xa - xb).magnitude()) * kr * kf);

			// Velocity Matching加速度作用在一起
			a_velMat += (V * (vb - va) * kr * kf);

			// Centering加速度作用在一起
			a_center += (C * (xb - xa) * kr * kf);

		}// particle b

		 // Acceleration Prioritization
		//计算出各个加速度的值是否超标
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
		//最后才能生成总的加速度
		Vector _aTotal = a_avoid + a_velMat + a_center;

		// if the boid is the lead boid
		if (i == leadBoidID) {
			Vector dir(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);//0~1转换为-0.5~0.5
			_aTotal = dir.unitvector() * _aTotal.magnitude();//随机改变它的方向
		}

		pq->set_accel(i, _aTotal);

	} // particle a
}

pba::Force pba::CreateAccumulatingBoidForce(const double A, const double V, const double C, const double Max, const double range, const double range_ramp)
{
	return pba::Force(new pba::AccumulatingBoidForce(A, V, C, Max, range, range_ramp));
}

//==========================================================================================

pba::Force pba::CreateAccumulatingGravityForce(const Vector & G)
{
	return pba::Force(new AccumulatingGravityForce(G));
}

/*===========================================SBD Spring&Friction=====================================================*/
//construction functions for
pba::Force pba::CreateAccumulatingStrutForce(const double G, const double f)
{
	return pba::Force(new AccumulatingStrutForce(G, f));
}

pba::Force pba::CreateAccumulatingStrutAreaForce(const double G, const double f)
{
	return pba::Force(new AccumulatingStrutAreaForce(G, f));
}

pba::Force pba::CreateAccumulatingStrutBendForce(const double G, const double f)
{
	return pba::Force(new AccumulatingStrutBendForce(G, f));
}

//all methods inside these classes
void pba::AccumulatingStrutForce::compute(pba::SoftBodyState & pq, const double dt)//长度和面积可以直接得到,所以这里重复计算了
{
	//===========================================cgforce=====================================
//	for (size_t i = 0; i < pq->nb_pairs(); i++)
//	{
//		const pba::SoftEdge& edge = pq->get_connected_pair(i);
//		const size_t& p1 = edge->get_first_node();
//		const size_t& p2 = edge->get_second_node();
//
//		const Vector& a = pq->pos(p1);
//		const Vector& b = pq->pos(p2);
//
//		Vector dir_ab = (a - b).unitvector();
//		Vector FSab = -spring * dir_ab * ((a - b).magnitude() - edge->get_edge_length());
//		Vector vel_ab = pq->vel(p1) - pq->vel(p2);
//		Vector FFab = -friction * dir_ab * ((a - b).unitvector() * vel_ab);
//
//		Vector Ftotal = FSab + FFab;
//		Vector aa = (Ftotal + pq->accel(p1) * pq->mass(p1)) / pq->mass(p1);
//		Vector ab = (pq->accel(p2) * pq->mass(p2) - Ftotal) / pq->mass(p2);
//
//// 		if ((a - b).magnitude() > pq->get_edge_threshold())
//// 			return;
//
//		pq->set_accel(p1, aa);
//		pq->set_accel(p2, ab);
//	}
	//=========================================================================================
	for (int i = 0; i < pq->nb_pairs(); i++)
	{
		//Spring force
		int indexi = pq->get_connected_pair(i)->get_first_node();
		int indexj = pq->get_connected_pair(i)->get_second_node();
		Vector temp = pq->pos(indexi) - pq->pos(indexj);
		double dis = temp.magnitude();
		Vector Xab = temp / dis;
		double home_dis = pq->get_connected_pair(i)->get_edge_length();
		Vector Fab_s = -1.0 * get_strength() * (dis - home_dis) * Xab;
		pq->set_accel(indexi, pq->accel(indexi) + Fab_s / pq->mass(indexi));
		pq->set_accel(indexj, pq->accel(indexj) - Fab_s / pq->mass(indexj));


		//Friction force
		Vector delVab = pq->vel(indexi) - pq->vel(indexj);
		Vector Fab_f = -1 * get_friction() * Xab *(Xab * delVab);
		pq->set_accel(indexi, pq->accel(indexi) + Fab_f / pq->mass(indexi));
		pq->set_accel(indexj, pq->accel(indexj) - Fab_f / pq->mass(indexj));
	}
	//=========================================================================================
	
	
}

void pba::AccumulatingStrutAreaForce::compute(pba::SoftBodyState & pq, const double dt)
{
		for (int i = 0; i < pq->nb_area_sets(); i++)
		{
			//Spring force
			int indexi = pq->get_area_set(i)->get_first_node();
			int indexj = pq->get_area_set(i)->get_second_node();
			int indexl = pq->get_area_set(i)->get_third_node();
			Vector temp = (pq->pos(indexj) - pq->pos(indexi)) ^ (pq->pos(indexl) - pq->pos(indexi));
			double area = temp.magnitude();//重复了!!!!!........................area = pq->get_area_set(i)->get_area();
			double home_area = pq->get_area_set(i)->get_area();
			//calculate d
			Vector tempd = pq->pos(indexi) - (pq->pos(indexj) + pq->pos(indexl)) / 2.0;
			Vector d0 = tempd / tempd.magnitude();//univector()
			tempd = pq->pos(indexj) - (pq->pos(indexi) + pq->pos(indexl)) / 2.0;
			Vector d1 = tempd / tempd.magnitude();
			tempd = pq->pos(indexl) - (pq->pos(indexi) + pq->pos(indexj)) / 2.0;
			Vector d2 = tempd / tempd.magnitude();

			Vector Fa_s = -1.0 * d0 * get_strength() * (area - home_area);
			pq->set_accel(indexi, pq->accel(indexi) + Fa_s / pq->mass(indexi));

			Fa_s = -1.0 * d1 * get_strength() * (area - home_area);
			pq->set_accel(indexj, pq->accel(indexj) + Fa_s / pq->mass(indexj));

			Fa_s = -1.0 * d2 * get_strength() * (area - home_area);
			pq->set_accel(indexl, pq->accel(indexl) + Fa_s / pq->mass(indexl));

			//friction force
			Vector tempv = pq->vel(indexi) - (pq->vel(indexj) + pq->vel(indexl)) / 2.0;
			Vector Fa_f = -1 * get_friction() * d0 * (d0 * tempv);
			pq->set_accel(indexi, pq->accel(indexi) + Fa_f / pq->mass(indexi));

			tempv = pq->vel(indexj) - (pq->vel(indexi) + pq->vel(indexl)) / 2.0;
			Fa_f = -1 * get_friction() * d1 * (d1 * tempv);
			pq->set_accel(indexj, pq->accel(indexj) + Fa_f / pq->mass(indexj));

			tempv = pq->vel(indexl) - (pq->vel(indexj) + pq->vel(indexi)) / 2.0;
			Fa_f = -1 * get_friction() * d2 * (d2 * tempv);
			pq->set_accel(indexl, pq->accel(indexl) + Fa_f / pq->mass(indexl));
		}
}

void pba::AccumulatingStrutBendForce::compute(pba::SoftBodyState & pq, const double dt)
{
	for (int i = 0; i < pq->nb_bendables(); i++)
	{
		int indexi = pq->get_bendable(i)->get_first_node();
		int indexj = pq->get_bendable(i)->get_second_node();
		int indexk = pq->get_bendable(i)->get_third_node();
		int indexl = pq->get_bendable(i)->get_fourth_node();

		//
		Vector e1 = pq->pos(indexj) - pq->pos(indexi);
		Vector e2 = pq->pos(indexk) - pq->pos(indexi);
		Vector f1 = pq->pos(indexj) - pq->pos(indexl);
		Vector f2 = pq->pos(indexk) - pq->pos(indexl);
		Vector temp = e1 ^ e2;
		Vector n0 = temp / temp.magnitude();
		temp = f2 ^ f1;
		Vector n1 = temp / temp.magnitude();
		temp = pq->pos(indexk) - pq->pos(indexj);
		Vector h = temp / temp.magnitude();

		double sint = h * (n0 ^ n1);
		double cost = n1 * n0;
		double t = atan2(sint, cost);

		double angle = pq->get_bendable(i)->get_value();
		//??????????????????get_strength()??????????????????/
		//Vector F0 = get_strength()*(angle - PI)*n0;
		Vector F0 = get_strength()*(t - angle)*n0;
		//Vector F3 = get_strength()*(angle - PI)*n1;
		Vector F3 = get_strength()*(t - angle)*n1;
		Vector X_cm;
		temp = pq->mass(indexi) * pq->pos(indexi) + pq->mass(indexj) * pq->pos(indexj) + pq->mass(indexk) * pq->pos(indexk) + pq->mass(indexl) * pq->pos(indexl);
		X_cm = temp / (pq->mass(indexi) + pq->mass(indexj) + pq->mass(indexk) + pq->mass(indexl));
		Vector r0 = pq->pos(indexi) - X_cm;
		Vector r1 = pq->pos(indexj) - X_cm;
		Vector r2 = pq->pos(indexk) - X_cm;
		Vector r3 = pq->pos(indexl) - X_cm;

		double q0 = (r1 - r2) * (r2 - r0) / ((r1 - r2) * (r1 - r2));
		double q3 = (r1 - r2) * (r2 - r3) / ((r1 - r2) * (r1 - r2));//r3??

		Vector F1 = q0 * F0 + q3 * F3;
		Vector F2 = -1 * (F1 + F0 + F3);

		pq->set_accel(indexi, pq->accel(indexi) + F0 / pq->mass(indexi));
		pq->set_accel(indexj, pq->accel(indexj) + F1 / pq->mass(indexj));
		pq->set_accel(indexk, pq->accel(indexk) + F2 / pq->mass(indexk));
		pq->set_accel(indexl, pq->accel(indexl) + F3 / pq->mass(indexl));
	}
	
}
