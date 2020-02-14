#pragma once
//-------------------------------------------------------
//
//  SPHState.h
//
//  Container for data associated with the dynamics
//  degrees of freedom of sph system.
//
//  Copyright (c) 2019 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_SPHSTATE_H____
#define ____PBA_SPHSTATE_H____

#include "DynamicalState.h"
//#include "OccupancyVolume.h"
#define PI 3.1415926


namespace pba
{


	class SPHStateData : public DynamicalStateData//, public OccupancyVolume
	{
	public:

		SPHStateData(/*const AABB& bounds,*/ const double h, const std::string& nam = "SPHDataNoName");
		SPHStateData(const SPHStateData& d);
		~SPHStateData();

		SPHStateData& operator= (const SPHStateData& d);

		const float get_radius() const { return radius; }
		void set_radius(const float& v);

		const float weight(size_t p, const Vector& P) const;
		const Vector grad_weight(size_t p, const Vector& P) const;//粒子a的序号和粒子b的位置向量

		void compute_density();//update density for all the particles,add up all P(b on a) for all a 
		void populate();


	private:

		float radius;


	};



	typedef std::shared_ptr<SPHStateData> SPHState;

	SPHState CreateSPH(/*const AABB& bounds,*/ const double h, const std::string& nam = "SPHDataNoName");

	SPHState copy(const SPHState d);//该方法在类外面




}
#endif
