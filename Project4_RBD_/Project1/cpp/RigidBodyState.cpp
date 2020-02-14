#include "..\include\RigidBodyState.h"

//construction
pba::RigidBodyStateData::RigidBodyStateData(const std::string & nam)//其实在solver里调用的时候再初始化也好；有些地方for循环可以合并这样只循环一次更新了多个变量提速？
{
	create_attr("r", Vector());
	create_attr("p", Vector());
	create_attr("torque", Vector());




	////float total mass
	//for (int i = 0; i < nb(); i++)
	//{
	//	total_mass += mass(i);
	//}
	//

	////Matrix angular_rotation;
	////angular_rotation = Matrix(1.0, .0, .0, .0, 1.0, .0, .0, .0, 1.0);//............?
	//angular_rotation = unitMatrix();//R                   //利用已有资源看得这种信息/zazhidao

	////Vector linear_velocity;
	//Vector tmp = Vector(0, 0, 0);
	//for (int i = 0; i < nb(); i++)
	//{
	//	tmp += mass(i)*vel(i);
	//}
	//linear_velocity = tmp / total_mass;//should be 0 at first

	////Vector angular_velocity;
	//angular_velocity = Vector(.0, .0, .0);//should be 0 at first

	////Vector center_of_mass_accel;
	//tmp = Vector(0, 0, 0);
	//for (int i = 0; i < nb(); i++)
	//{
	//	tmp += mass(i)*accel(i);
	//}
	//center_of_mass_accel = tmp / total_mass;

	////Vector angular_accel;
	//angular_accel = Vector(.0, .0, .0);

	////Vector angular_momentum;
	//angular_momentum = moment_of_inertia * angular_velocity;//I* w = L


	////Matrix moment_of_inertia;
	////===============      two way to get it?         =============================//
	///*for (int i = 0; i < 3; i++)
	//{
	//	for (int j; j < 3; j++)
	//	{
	//		for (int k = 0; k < nb(); k++)
	//		{
	//			if (i == j)
	//				moment_of_inertia[i][j] += mass(k) * ((pos(k) - center_of_mass).magnitude() * center_of_mass).magnitude() - (pos(k) - center_of_mass).(i) * (pos(k) - center_of_mass).[j]         );
	//			else if (i != j)
	//				moment_of_inertia[i][j] += mass(k) * (0.0 - (pos(k) - center_of_mass).(i) * (pos(k) - center_of_mass).[j]);
	//			
	//		}
	//	}
	//}*/
	//Matrix M;
	//Vector r;
	//for (int i = 0; i < nb(); i++)
	//{
	//	r = pos(i) - center_of_mass;
	//	outer_product(r, r, M);
	//	moment_of_inertia += mass(i) *  (r * r * unitMatrix() - M);
	//}

	////Matrix inverse_moment_of_inertia;
	//inverse_moment_of_inertia = inverse(moment_of_inertia);//...............?



	//compute_RBD_data();
}

//construct 2
pba::RigidBodyStateData::RigidBodyStateData(const RigidBodyStateData & d)
{
	center_of_mass = d.center_of_mass;
	angular_rotation = d.angular_rotation;
	linear_velocity = d.linear_velocity;
	angular_velocity = d.angular_velocity;
	center_of_mass_accel = d.center_of_mass_accel;
	angular_accel = d.angular_accel;
	angular_momentum = d.angular_momentum;

	moment_of_inertia = d.inertia_moment();
	inverse_moment_of_inertia = d.inverse_moi();
	total_mass = d.totalmass();
}

pba::RigidBodyStateData::~RigidBodyStateData()
{
}


//===============================================================================================
//Initialization before solver
void pba::RigidBodyStateData::compute_RBD_data()
{
	//Vector homePos;
	//for (int i = 0; i < nb(); i++) {
	//	homePos = pos(i);
	//	set_attr("p", i, homePos);
	//}
	////vector center_of_mass;
	//Vector tmp = Vector(0, 0, 0);
	//for (int i = 0; i < nb(); i++)
	//{
	//	tmp += mass(i)*pos(i);
	//}
	//center_of_mass = tmp / total_mass;

	////r 相对位置
	//for (int i = 0; i < nb(); i++)
	//{
	//	Vector r = pos(i) - center_of_mass;
	//	set_attr("r", i, r);
	//}

	for (int i = 0; i < nb(); i++) {
		Vector homePos = pos(i);
		set_attr("p", i, homePos);
	}

	angular_rotation = unitMatrix();

	total_mass = 0;
	Vector _p, _v;
	for (int i = 0; i < nb(); i++)
	{
		total_mass += mass(i);
		_p += (mass(i) * pos(i));
		_v += (mass(i) * vel(i));
	}

	center_of_mass = _p / total_mass;
	linear_velocity = Vector();
	angular_velocity = Vector();

	for (int i = 0; i < nb(); i++)
	{
		Vector r = pos(i) - center_of_mass;
		set_attr("r", i, r);
	}

	// Update the moment_of_inertia
	recompute_MOI();

	angular_momentum = moment_of_inertia * angular_velocity;

}

void pba::RigidBodyStateData::compute_M()
{
}

void pba::RigidBodyStateData::recompute_MOI()
{
	Matrix M;
	Vector r;
	for (int i = 0; i < nb(); i++)
	{
		//r = pos(i) - center_of_mass;
		r = get_vector_attr("r", i);
		outer_product(r, r, M);//没存起来浪费时间
		moment_of_inertia += mass(i) *  (r * r * unitMatrix() - M);
	}

	//Matrix inverse_moment_of_inertia;
	inverse_moment_of_inertia = inverse(moment_of_inertia);
}

double pba::RigidBodyStateData::total_energy() const
{
	return 0.0;
}

pba::Vector pba::RigidBodyStateData::vert_pos(const size_t p) const
{
	return Vector();
}

pba::RigidBodyState pba::CreateRigidBody(const std::string & nam)
{
	return RigidBodyState(new RigidBodyStateData(nam));
}
