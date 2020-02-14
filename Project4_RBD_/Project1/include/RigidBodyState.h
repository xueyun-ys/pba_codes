#pragma once
//-------------------------------------------------------
//
//  RigidBodyState.h
//
//  Container for data associated with the dynamics
//  degrees of freedom of a single rigid body.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_RIGIDBODYSTATE_H____
#define ____PBA_RIGIDBODYSTATE_H____

#include "DynamicalState.h"
#include "Matrix.h"
#include "LinearAlgebra.h"


namespace pba
{


	class RigidBodyStateData : public DynamicalStateData
	{
	public:

		RigidBodyStateData(const std::string& nam = "RBDDataNoName");
		RigidBodyStateData(const RigidBodyStateData& d);
		~RigidBodyStateData();

		RigidBodyStateData& operator= (const RigidBodyStateData& d);

		void compute_RBD_data();
		void compute_M();
		void recompute_MOI();

		const Matrix& inertia_moment() const { return moment_of_inertia; }
		const Matrix& inverse_moi() const { return  inverse_moment_of_inertia; }
		const float totalmass() const { return total_mass; }//for private variables

		const Matrix& raw_moi() const { return moment_of_inertia; }//...?

		double total_energy() const;

		Vector vert_pos(const size_t p) const;

		Vector center_of_mass;
		Matrix angular_rotation;
		Vector linear_velocity;//Vcm
		Vector angular_velocity;
		Vector center_of_mass_accel;
		Vector angular_accel;
		Vector angular_momentum;

	private:

		Matrix moment_of_inertia;
		Matrix inverse_moment_of_inertia;
		float total_mass;
	};



	typedef std::shared_ptr<RigidBodyStateData> RigidBodyState;

	RigidBodyState CreateRigidBody(const std::string& nam = "RigidBodyDataNoName");

	RigidBodyState copy(const RigidBodyState d);

}
#endif
