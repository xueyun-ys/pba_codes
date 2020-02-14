#pragma once
//-------------------------------------------------------
//
//  TorqueLibrary.h
//
//  A collection of torque models
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_TORQUELIBRARY_H____
#define ____PBA_TORQUELIBRARY_H____

#include "Torque.h"


namespace pba
{


	class TorqueFromForce : public TorqueBase
	{
	public:

		TorqueFromForce(Force& f) :
			force(f)
		{}

		~TorqueFromForce() {}

		void compute(RigidBodyState& s, const double dt);

	private:

		Force force;

	};

	Torque CreateTorqueFromForce(Force& f);



}
#endif
