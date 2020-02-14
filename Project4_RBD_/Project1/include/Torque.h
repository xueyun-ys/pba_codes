#pragma once
//-------------------------------------------------------
//
//  Torque.h
//
//  Base class and refcounted pointer for torque
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_TORQUE_H____
#define ____PBA_TORQUE_H____

#include "RigidBodyState.h"
#include "Force.h"
#include <iostream>


namespace pba
{


	class TorqueBase
	{
	public:

		TorqueBase() {};

		virtual void compute(RigidBodyState& s, const double dt) { std::cout << "calling TorqueBase::compute(RigidBodyState,double) base class virtual method\n"; }
		//virtual void compute(SoftBodyState& s, const double dt) { std::cout << "calling TorqueBase::compute(SoftBodyState,double) base class virtual method\n"; }
		virtual ~TorqueBase() {};

	};



	typedef std::shared_ptr<TorqueBase> Torque;





}
#endif