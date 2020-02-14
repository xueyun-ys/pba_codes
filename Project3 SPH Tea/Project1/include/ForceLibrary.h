#pragma once
//-------------------------------------------------------
//
//  ForceLibrary.h
//
//  A collection of force implementations
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_FORCELIBRARY_H____
#define ____PBA_FORCELIBRARY_H____

#include "Force.h"

namespace pba
{



	class AccumulatingForce : public ForceBase
	{
	public:

		AccumulatingForce() {}

		~AccumulatingForce() {}

		void compute(DynamicalState& s, const double dt);
		//void compute(RigidBodyState& s, const double dt);
		//void compute(SoftBodyState& s, const double dt);
		void compute(SPHState& s, const double dt);

		void add(Force& f);


	private:

		std::vector<Force> forces;//开头字母小写的vector表示向量using a vector to contain all the force and compute each of them by void compute(){};

	};

	Force CreateAccumulatingForce();


	/*class AccumulatingHarmonicForce : public ForceBase
	{
	public:

		AccumulatingHarmonicForce(const double k, const Vector& pos) :
			spring_constant(k),
			rest_position(pos)
		{}

		~AccumulatingHarmonicForce() {}

		void compute(DynamicalState& s, const double dt);
		void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}

		void set_spring_constant(const double v) { spring_constant = v; }
		const float get_spring_constant() const { return spring_constant; }

		void set_rest_position(const Vector& v) { rest_position = v; }
		const Vector& get_rest_position() const { return rest_position; }

	private:

		double spring_constant;
		Vector rest_position;

	};

	Force CreateAccumulatingHarmonicForce(const double k, const Vector& pos);*/





	class AccumulatingBoidCollisionAvoidanceForce : public pba::ForceBase
	{

	public:

		AccumulatingBoidCollisionAvoidanceForce(const double g) : G(g) {}
		~AccumulatingBoidCollisionAvoidanceForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		/*void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}*/


		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

	private:

		double G;
	};

	pba::Force CreateAccumulatingBoidCollisionAvoidanceForce(const double G);




	class AccumulatingBoidVelocityMatchingForce : public pba::ForceBase
	{

	public:

		AccumulatingBoidVelocityMatchingForce(const double g) : G(g) {}
		~AccumulatingBoidVelocityMatchingForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		/*void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}*/


		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

	private:

		double G;
	};

	pba::Force CreateAccumulatingBoidVelocityMatchingForce(const double G);





	class AccumulatingBoidCenteringForce : public pba::ForceBase
	{

	public:

		AccumulatingBoidCenteringForce(const double g) : G(g) {}
		~AccumulatingBoidCenteringForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		/*void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}*/


		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

	private:

		double G;
	};

	pba::Force CreateAccumulatingBoidCenteringForce(const double G);





	class AccumulatingBoidForce : public pba::ForceBase
	{

	public:

		AccumulatingBoidForce(const double a, const double v, const double c, const double Max, const double rng, const double rng_ramp = 1.0) :
			A(a),
			V(v),
			C(c),
			amax(Max),
			range(rng),
			range_ramp(rng_ramp),
			fov(270.0),
			dfov(10.0)
		{
			cosfov = std::cos(fov*3.14159265 / 360.0);

		}

		~AccumulatingBoidForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		/*void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}*/


		void set_avoidance(const double v) { A = v; };
		const double get_avoidance() const { return A; };

		void set_matching(const double v) { V = v; };
		const double get_matching() const { return V; };

		void set_centering(const double v) { C = v; };
		const double get_centering() const { return C; };

		void set_max(const double v) { amax = v; };
		const double get_max() const { return amax; };

		void set_range(const double v) { range = v; };
		const double get_range() const { return range; };

		void set_range_ramp(const double v) { range_ramp = v; };
		const double get_range_ramp() const { return range_ramp; };

		void set_fov(const double v) { fov = v; if (fov > 360.0) { fov = 360.0; } cosfov = std::cos(v*3.14159265 / 360.0); set_fov_shell(dfov); };
		const double get_fov() const { return fov; };

		void set_fov_shell(const double v) { dfov = v; };//...{.}
		const double get_fov_shell() const { return dfov; };

		//...
		void set_lead_boid(int _id) { leadBoidID = _id; }

	private:

		double A, V, C;
		double amax;
		double range, range_ramp;
		double fov;//较大视野
		double dfov;//较小视野
		double cosfov;
		double cosfovshell;
		//...w
		int leadBoidID;
	};

	pba::Force CreateAccumulatingBoidForce(const double A, const double V, const double C, const double Max, const double range, const double range_ramp);








	/*class AccumulatingMagneticDipoleForce : public pba::ForceBase
	{

	public:

		AccumulatingMagneticDipoleForce(const double g, const Vector& m, const Vector& P) :
			G(g),
			dipole_moment(m),
			dipole_location(P)
		{}

		~AccumulatingMagneticDipoleForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}


		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

		void set_dipole_moment(const Vector& v) { dipole_moment = v; };
		const Vector& get_dipole_moment() const { return dipole_moment; };

		void set_dipole_location(const Vector& v) { dipole_location = v; };
		const Vector& get_dipole_location() const { return dipole_location; };

	private:

		double G;
		Vector dipole_moment;
		Vector dipole_location;
	};

	pba::Force CreateAccumulatingMagneticDipoleForce(const double G, const Vector& m, const Vector& P);





	class AccumulatingMagneticMonopoleForce : public pba::ForceBase
	{

	public:

		AccumulatingMagneticMonopoleForce(const double g, const double q, const Vector& P) :
			G(g),
			monopole_power(q),
			monopole_location(P)
		{}

		~AccumulatingMagneticMonopoleForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}


		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

		void set_monopole_power(const double v) { monopole_power = v; };
		const double get_monopole_power() const { return monopole_power; };

		void set_monopole_location(const Vector& v) { monopole_location = v; };
		const Vector& get_monopole_location() const { return monopole_location; };

	private:

		double G;
		double monopole_power;
		Vector monopole_location;
	};

	pba::Force CreateAccumulatingMagneticMonopoleForce(const double G, const double q, const Vector& P);*/






	class AccumulatingGravityForce : public pba::ForceBase
	{

	public:

		AccumulatingGravityForce(const Vector& g) :
			G(g)
		{}

		~AccumulatingGravityForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		/*void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SPHState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SPHStateData>(s);
			compute(ss, dt);
		}*/



		void set_strength(const double v) { G *= v / G.magnitude(); };
		const double get_strength() const { return G.magnitude(); };

	private:

		Vector G;
	};

	pba::Force CreateAccumulatingGravityForce(const Vector& G);





	/*class AccumulatingLeonardJonesForce : public pba::ForceBase
	{

	public:

		AccumulatingLeonardJonesForce(const double g, const double ro, const Vector& c) :
			G(g),
			r0(ro),
			center(c)
		{}

		~AccumulatingLeonardJonesForce() {};

		void compute(pba::DynamicalState& pq, const double dt);
		void compute(RigidBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(s);
			compute(ss, dt);
		}
		void compute(SoftBodyState& s, const double dt)
		{
			DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, SoftBodyStateData>(s);
			compute(ss, dt);
		}


		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

		void set_gap_range(const double v) { r0 = v; };
		const double get_gap_range() const { return r0; };

	private:

		double G;
		double r0;
		Vector center;
	};

	pba::Force CreateAccumulatingLeonardJonesForce(const double G, const double ro, const Vector& c);







	class AccumulatingStrutForce : public pba::ForceBase
	{

	public:

		AccumulatingStrutForce(const double g, const double f) :
			spring(g),
			friction(f)
		{}

		~AccumulatingStrutForce() {};

		void compute(pba::SoftBodyState& pq, const double dt);

		void set_strength(const double v) { spring = v; };
		const double get_strength() const { return spring; };

		void set_friction(const double v) { friction = v; };
		const double get_friction() const { return friction; };

	private:

		double spring;
		double friction;
	};

	pba::Force CreateAccumulatingStrutForce(const double G, const double f);




	class AccumulatingStrutAreaForce : public pba::ForceBase
	{

	public:

		AccumulatingStrutAreaForce(const double g, const double f) :
			spring(g),
			friction(f)
		{}

		~AccumulatingStrutAreaForce() {};

		void compute(pba::SoftBodyState& pq, const double dt);

		void set_strength(const double v) { spring = v; };
		const double get_strength() const { return spring; };

		void set_friction(const double v) { friction = v; };
		const double get_friction() const { return friction; };

	private:

		double spring;
		double friction;
	};

	pba::Force CreateAccumulatingStrutAreaForce(const double G, const double f);





	class AccumulatingStrutBendForce : public pba::ForceBase
	{

	public:

		AccumulatingStrutBendForce(const double g, const double f) :
			spring(g),
			friction(f)
		{}

		~AccumulatingStrutBendForce() {};

		void compute(pba::SoftBodyState& pq, const double dt);

		void set_strength(const double v) { spring = v; };
		const double get_strength() const { return spring; };

		void set_friction(const double v) { friction = v; };
		const double get_friction() const { return friction; };

	private:

		double spring;
		double friction;
	};

	pba::Force CreateAccumulatingStrutBendForce(const double G, const double f);*/
























	/*class HarmonicForce : public ForceBase
	{
	public:

		HarmonicForce(const double k, const Vector& pos) :
			spring_constant(k),
			rest_position(pos)
		{}

		~HarmonicForce() {}

		void compute(DynamicalState& s, const double dt);

		void set_spring_constant(const double v) { spring_constant = v; }
		const float get_spring_constant() const { return spring_constant; }

		void set_rest_position(const Vector& v) { rest_position = v; }
		const Vector& get_rest_position() const { return rest_position; }

	private:

		double spring_constant;
		Vector rest_position;

	};

	Force CreateHarmonicForce(const double k, const Vector& pos);




	class AnharmonicForce : public ForceBase
	{
	public:

		AnharmonicForce(const double k, const double w, const Vector& pos) :
			spring_constant(k),
			well_constant(w),
			rest_position(pos)
		{}

		~AnharmonicForce() {}

		void compute(DynamicalState& s, const double dt);

		void set_spring_constant(const double v) { spring_constant = v; }
		const float get_spring_constant() const { return spring_constant; }

		void set_well_constant(const double v) { well_constant = v; }
		const float get_well_constant() const { return well_constant; }

		void set_rest_position(const Vector& v) { rest_position = v; }
		const Vector& get_rest_position() const { return rest_position; }

	private:

		double spring_constant;
		double well_constant;
		Vector rest_position;

	};

	Force CreateAnharmonicForce(const double k, const double w, const Vector& pos);







	class CoupledOscillatorForce : public pba::ForceBase
	{

	public:

		CoupledOscillatorForce(const double g) : G(g) {}
		~CoupledOscillatorForce() {}

		void compute(pba::DynamicalState& pq, const double dt);

		void set_spring_constant(const double v) { G = v; }
		const double& get_spring_constant() const { return G; }

	private:

		double G;
	};

	pba::Force CreateCoupledOscillatorForce(const double G);
*/





	class GravitationalForce : public pba::ForceBase
	{

	public:

		GravitationalForce(const double g) : G(g) {}
		~GravitationalForce() {}

		void compute(pba::DynamicalState& pq, const double dt);

		void set_gravitational_constant(const double v) { G = v; }
		const double& get_gravitational_constant() const { return G; }

	private:

		double G;
	};

	pba::Force CreateGravitationalForce(const double G);





	class AccumulatingAcceleratedGravitationalForce : public pba::ForceBase
	{

	public:

		AccumulatingAcceleratedGravitationalForce(const double g, const double cellsize) : G(g), h(cellsize) {}
		~AccumulatingAcceleratedGravitationalForce() {}

		void compute(pba::DynamicalState& pq, const double dt);

		void set_gravitational_constant(const double v) { G = v; }
		const double& get_gravitational_constant() const { return G; }

		void set_cell_spacing(const double v) { h = v; }
		const double& get_cell_spacing() const { return h; }

	private:

		double G;
		double h;
	};

	pba::Force CreateAccumulatingAcceleratedGravitationalForce(const double G, const double h);









	/*class SpringyForce : public ForceBase
	{
	public:

		SpringyForce(const double k, const Vector& pos, const double g) :
			spring_constant(k),
			rest_position(pos),
			G(Vector(0, -1, 0)*g)
		{}

		~SpringyForce() {}

		void compute(DynamicalState& s, const double dt);

		void set_spring_constant(const double v) { spring_constant = v; }
		const float get_spring_constant() const { return spring_constant; }

		void set_rest_position(const Vector& v) { rest_position = v; }
		const Vector& get_rest_position() const { return rest_position; }

		void set_gravitational_constant(const Vector& v) { G = v; }
		const Vector& get_gravitational_constant() const { return G; }

	private:

		double spring_constant;
		Vector rest_position;
		Vector G;

	};

	Force CreateSpringyForce(const double k, const Vector& pos, const double g);






	class NonlinearHarmonicOscillatorForce : public pba::ForceBase
	{

	public:

		NonlinearHarmonicOscillatorForce(const double g, const double friction, const double pert, const double driver, const double omega) :
			G(g),
			Friction(friction),
			Perturbation(pert),
			Driver(driver),
			Omega(omega)
		{};

		~NonlinearHarmonicOscillatorForce() {};

		void compute(pba::DynamicalState& pq, const double dt);

		void set_spring_constant(const double v) { G = v; }
		const double& get_spring_constant() const { return G; }

		void set_driver_constant(const double v) { Driver = v; }
		const double& get_driver_constant() const { return Driver; }

		void set_frequency_constant(const double v) { Omega = v; }
		const double& get_frequency_constant() const { return Omega; }

		void set_perturbation_constant(const double v) { Perturbation = v; }
		const double& get_perturbation_constant() const { return Perturbation; }

		void set_friction_constant(const double v) { Friction = v; }
		const double& get_friction_constant() const { return Friction; }

	private:

		double G;
		double Friction;
		double Perturbation;
		double Driver;
		double Omega;
	};

	pba::Force CreateNonlinearHarmonicOscillatorForce(const double G, const double F, const double P, const double D, const double O);*/




	class SimpleGravityForce : public pba::ForceBase
	{

	public:

		SimpleGravityForce(const pba::Vector& g) : G(g) {}//参数：重力的方向向量
		~SimpleGravityForce() {}

		void compute(pba::DynamicalState& pq, const double dt);

		void set_gravity_constant(const double v) { G = G.unitvector() * v; }
		const double get_gravity_constant() const { return G.magnitude(); }

	private:

		pba::Vector G;
	};

	pba::Force CreateSimpleGravityForce(const pba::Vector& G);



	/*class RandomWalkForce : public pba::ForceBase
	{

	public:

		RandomWalkForce(const double g = 1.0) : G(g) {}
		~RandomWalkForce() {};

		void compute(pba::DynamicalState& pq, const double dt);

		void set_strength(const double v) { G = v; }
		const double get_strength() const { return G; }

	private:

		double G;
	};

	pba::Force CreateRandomWalkForce(const double G);





	class CorrelatedRandomWalkForce : public pba::ForceBase
	{

	public:

		CorrelatedRandomWalkForce(const double g, const double corr) : G(g), epsilon(corr) {}
		~CorrelatedRandomWalkForce() {};

		void compute(pba::DynamicalState& pq, const double dt);

		void set_strength(const double v) { G = v; };
		const double get_strength() const { return G; };

		void set_correlation(const double v) { epsilon = v; };
		const double get_correlation() const { return epsilon; };

	private:

		double G;
		double epsilon;
	};

	pba::Force CreateCorrelatedRandomWalkForce(const double G, const double corr);
	*/















}
#endif
