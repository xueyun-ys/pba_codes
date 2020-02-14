#include "SPHForce.h"

//pba::SPHForce::SPHForce()
//{
//	epsilon_sph = 0.1f;
//	pressure_magnitude = 1.0f;
//	pressure_power = 3.0f;
//	pressure_base = 20.0f;
//	alpha_sph = 0.5f;
//	beta_sph = 0.5f;
//}

void pba::SPHForce::compute(SPHState& s, const double dt)
{
	compute_pressure(s);

#pragma omp parallel for
	for (int i = 0; i < s->nb(); i++)
	{
		Vector viscosityF = Vector(0.0, 0.0, 0.0);
		Vector pos_a = s->pos(i);
		Vector vel_a = s->vel(i);
#pragma omp parallel for
		for (int j = 0; j < s->nb(); j++)
		{
			if (j == i)
				continue;

			Vector pos_b = s->pos(j);
			Vector vel_b = s->vel(j);

			double mass_b = s->mass(j);

			double _e = (vel_a - vel_b) * (pos_a - pos_b);
			double _p;

			if (_e >= 0)
				_p = 0.0;
			else
			{
				double den_a = s->get_float_attr("den", i);
				double den_b = s->get_float_attr("den", j);
				double _den = (den_a + den_b) / 2;

				if (_den == 0.0)
					continue;

				double h = s->get_radius() / 2;
				double _pos = (pos_a - pos_b).magnitude();
				double _u = (h * _e) / ((_pos * _pos) + (epsilon_sph * h * h));

				float sound_a = sound_speed(den_a);
				float sound_b = sound_speed(den_b);
				float _c = (sound_a + sound_b) / 2;

				_p = ((-alpha_sph * _c + beta_sph * (_u * _u)) / _den);
			}
			viscosityF -= mass_b * _p * s->grad_weight(i, s->pos(j));
		}

		s->set_accel(i, s->accel(i) + (viscosityF / s->mass(i)));
	}
}

void pba::SPHForce::compute_pressure(SPHState& s)
{
#pragma omp parallel for
	for (int i = 0; i < s->nb(); i++)
	{
		Vector pressureF;
		float den_a = s->get_float_attr("den", i);

		if (den_a == 0.0f)
			continue;

		float pres_a = pressure_magnitude * (pow((den_a / pressure_base), pressure_power) - 1);
#pragma omp parallel for
		for (int j = 0; j < s->nb(); j++)
		{
			if (j == i)
				continue;

			float m_b = s->mass(j);
			float den_b = s->get_float_attr("den", j);

			if (den_b == 0.0f)
				continue;

			float pres_b = pressure_magnitude * (pow((den_b / pressure_base), pressure_power) - 1);
			Vector ureF = (m_b * ((pres_a / (den_a * den_a) + pres_b / (den_b * den_b))) * s->grad_weight(i, s->pos(j)));
			pressureF -= (m_b * ((pres_a / (den_a * den_a) + pres_b / (den_b * den_b))) * s->grad_weight(i, s->pos(j)));
		}

		Vector tmpAccel = (pressureF / s->mass(i));
		s->set_accel(i, s->accel(i) + (pressureF / s->mass(i)));
	}
}

//void pba::SPHForce::compute_viscosity(SPHState& s)
//{
//
//}

float pba::SPHForce::sound_speed(const float density) const
{
	return ((pressure_power * pressure_magnitude) / pressure_base) * pow((density / pressure_base), (pressure_power - 1));
}

pba::Force pba::CreateSPHForce()
{
	return Force(new SPHForce());
}
//cgcod abov====================
//#include "SPHForce.h"//复制的话需要调整调用路径
//#include "Force.h"
//
//pba::Force pba::CreateSPHForce()
//{
//	return Force(new SPHForce());
//}
//
//void pba::SPHForce::compute(SPHState & s, const double dt)//compute Viscosity
//{
//	float Eab = 0.0;
//	float Pi_ab = 0.0;
//	float den_ab = 0.0;
//	float uab = 0.0;
//	float Cab = 0.0;
//	float h = s->get_radius() / 2.0;
//	float Ca, Cb;
//	Vector F_visco = Vector(.0,.0,.0);
//	Vector temF;
//
//	for (int i = 0; i < s->nb(); i++)
//	{
//		for (int j = 0; j < s->nb(); j++)
//		{
//			if (i == j)
//				continue;
//			else if (i != j)
//			{
//				//if(s->pos(i) - s->pos(j) == Vector(.0,.0,.0))
//				if (s->pos(i) == s->pos(j))
//				{
//					continue;
//					Eab = (s->vel(i) - s->vel(j)) * Vector(0.001, 0.001, 0.001);//how to deal with that?????
//				}
//				else
//				{
//					Eab = (s->vel(i) - s->vel(j)) * (s->pos(i) - s->pos(j));
//				}
//				
//				den_ab = (s->get_float_attr("den", i) + s->get_float_attr("den", j)) / 2.0;
//				uab = h * Eab / ((s->pos(i) - s->pos(j)) * (s->pos(i) - s->pos(j)) + epsilon_sph * h *h);
//				Ca = pressure_power * pressure_magnitude / pressure_base * pow(s->get_float_attr("den", i) / pressure_base, pressure_power - 1.0);
//				Cb = pressure_power * pressure_magnitude / pressure_base * pow(s->get_float_attr("den", j) / pressure_base, pressure_power - 1.0);
//				Cab = (Ca + Cb) / 2.0;
//
//				if (Eab >= 0)
//				{
//					Pi_ab = 0.0;
//				}
//				else if (Eab < 0.0)
//				{
//					Pi_ab = (-1.0 * alpha_sph * Cab + beta_sph * uab * uab) / den_ab;
//				}
//				temF = s->mass(j) * Pi_ab /** Vector(1.0, 1.0, 1.0)*/* s->grad_weight(i, s->pos(j));
//				F_visco -= temF;
//			}
//		}
//		s->set_accel(i, s->accel(i) + (F_visco / s->mass(i)));
//	}
//
//	SPHForce::compute_pressure(s);
//}
//
//void pba::SPHForce::compute_pressure(SPHState & s)
//{
//	for (int i = 0; i < s->nb(); i++)
//	{
//		Vector pressureF = Vector(0.0, 0.0, 0.0);
//		float den_pa = s->get_float_attr("den", i);
//		float pres_a = pressure_magnitude * (pow((den_pa / pressure_base), pressure_power) - 1);
//		for (int j = 0; j < s->nb(); j++)
//		{
//			if (j == i)
//				continue;
//
//			float m_b = s->mass(j);
//			float den_pb = s->get_float_attr("den", j);
//			float pres_b = pressure_magnitude * (pow((den_pb / pressure_base), pressure_power) - 1);
//			Vector test;
//			test = s->grad_weight(i, s->pos(j));
//			pressureF -= (     m_b   * (  pres_a / (den_pa * den_pa) + pres_b / (den_pb * den_pb)  ) *   s->grad_weight(i, s->pos(j))       ) ;
//		}
//
//		s->set_accel(i, s->accel(i) + (pressureF / s->mass(i)));
//	}
//}
//
//float pba::SPHForce::sound_speed(const float density) const
//{
//	float Ca = pressure_power * pressure_magnitude / pressure_base * pow(density / pressure_base, pressure_power - 1.0);
//	return Ca;
//}
