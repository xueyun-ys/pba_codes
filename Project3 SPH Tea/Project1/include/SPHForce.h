#pragma once
//-------------------------------------------------------
//
//  SPHForce.h
//
//  A collection of force implementations for SPH
//
//  Copyright (c) 2019 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_SPHFORCE_H____
#define ____PBA_SPHFORCE_H____

#include "Force.h"
#include "SPHState.h"

namespace pba
{


	class SPHForce : public ForceBase
	{
	public:

		SPHForce() 
		{
			epsilon_sph = 0.1f;
			pressure_magnitude = 1.0f;
			pressure_power = 3.0f;
			pressure_base = 20.0f;//reference density;
			alpha_sph = 0.5f;
			beta_sph = 0.5f;
		};
		~SPHForce() {};

		void compute(SPHState& s, const double dt);

		void compute_pressure(SPHState& s);

		const float& get_pressure_base() const { return pressure_base; }
		const float& get_pressure_magnitude() const { return pressure_magnitude; }
		const float& get_pressure_power() const { return pressure_power; }
		const float& get_epsilon_sph() const { return epsilon_sph; }
		const float& get_alpha_sph() const { return alpha_sph; }
		const float& get_beta_sph() const { return beta_sph; }

		void set_pressure_base(const float v) { pressure_base = v; }
		void set_pressure_magnitude(const float v) { pressure_magnitude = v; }
		void set_pressure_power(const float v) { pressure_power = v; }
		void set_epsilon_sph(const float v) { epsilon_sph = v; }
		void set_alpha_sph(const float v) { alpha_sph = v; }
		void set_beta_sph(const float v) { beta_sph = v; }

	private:

		float pressure_magnitude = 1.0;
		float pressure_power = 3.0;
		float pressure_base = 2.0f;

		float epsilon_sph = 0.1;
		float alpha_sph = 1.0;
		float beta_sph = 1.0;

		float sound_speed(const float density) const;

	};

	Force CreateSPHForce();






}
#endif
