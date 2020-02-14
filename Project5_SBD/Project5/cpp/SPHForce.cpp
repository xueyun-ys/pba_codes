#include "SPHForce.h"//复制的话需要调整调用路径
#include "Force.h"

pba::Force pba::CreateSPHForce()
{
	return Force(new SPHForce());
}

void pba::SPHForce::compute(SPHState & s, const double dt)//compute Viscosity
{
	float Eab = 0.0;
	float Pi_ab = 0.0;
	float den_ab = 0.0;
	float uab = 0.0;
	float Cab = 0.0;
	float h = s->get_radius() / 2.0;
	float Ca, Cb;
	Vector F_visco = Vector(.0,.0,.0);
	Vector temF;

	for (int i = 0; i < s->nb(); i++)
	{
		for (int j = 0; j < s->nb(); j++)
		{
			if (i != j)
			{
				//if(s->pos(i) - s->pos(j) == Vector(.0,.0,.0))
				if (s->pos(i) == s->pos(j))
				{
					Eab = (s->vel(i) - s->vel(j)) * Vector(0.001, 0.001, 0.001);//how to deal with that?????
				}
				else
				{
					Eab = (s->vel(i) - s->vel(j)) * (s->pos(i) - s->pos(j));
				}
				
				den_ab = (s->get_float_attr("den", i) + s->get_float_attr("den", j)) / 2.0;
				uab = h * Eab / ((s->pos(i) - s->pos(j)) * (s->pos(i) - s->pos(j)) + epsilon_sph * h *h);
				Ca = pressure_power * pressure_magnitude / pressure_base * pow(s->get_float_attr("den", i) / pressure_base, pressure_power - 1.0);
				Cb = pressure_power * pressure_magnitude / pressure_base * pow(s->get_float_attr("den", j) / pressure_base, pressure_power - 1.0);
				Cab = (Ca + Cb) / 2.0;

				if (Eab >= 0)
				{
					Pi_ab = 0.0;
				}
				else if (Eab < 0.0)
				{
					Pi_ab = (-1.0 * alpha_sph * Cab + beta_sph * uab * uab) / den_ab;
				}
				temF = s->mass(j) * Pi_ab /** Vector(1.0, 1.0, 1.0)*/* s->grad_weight(i, s->pos(j));
				F_visco -= temF;
			}
		}
		s->set_accel(i, s->accel(i) + (F_visco / s->mass(i)));
	}

	SPHForce::compute_pressure(s);
}

void pba::SPHForce::compute_pressure(SPHState & s)
{
	for (int i = 0; i < s->nb(); i++)
	{
		Vector pressureF = Vector(0.0, 0.0, 0.0);
		float den_pa = s->get_float_attr("den", i);
		float pres_a = pressure_magnitude * (pow((den_pa / pressure_base), pressure_power) - 1);
		for (int j = 0; j < s->nb(); j++)
		{
			if (j == i)
				continue;

			float m_b = s->mass(j);
			float den_pb = s->get_float_attr("den", j);
			float pres_b = pressure_magnitude * (pow((den_pb / pressure_base), pressure_power) - 1);
			Vector test;
			test = s->grad_weight(i, s->pos(j));
			pressureF -= (     m_b   * (  pres_a / (den_pa * den_pa) + pres_b / (den_pb * den_pb)  ) *   s->grad_weight(i, s->pos(j))       ) ;
		}

		s->set_accel(i, s->accel(i) + (pressureF / s->mass(i)));
	}
}

float pba::SPHForce::sound_speed(const float density) const
{
	float Ca = pressure_power * pressure_magnitude / pressure_base * pow(density / pressure_base, pressure_power - 1.0);
	return Ca;
}
