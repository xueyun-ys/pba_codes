#include "CollisionHandler.h"

void pba::CollisionHandler::set_collision_surface(CollisionSurface& c)
{
	surf = c;
}

//===================================general collision================================================
void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& S)//Explicit中的advance的solve handler-surface-triangle;dt = delta t
{
	pba::CollisionData* CD = new pba::CollisionData;
	double vn;//
	Vector vp, vr;//
	for (int i = 0; i < S->nb(); i++)
	{
		CD->hit_index = i;
		if (surf->hit(S->pos(i), S->vel(i), dt, *CD))//进入下一层，triangle
		{
			//update velocity and positions
			Vector v = S->vel(i);
			Vector norm = CD->tri->N();
			vn = norm * S->vel(i);
			vp = S->vel(i) - norm * vn;
			vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);//For Elastic collision

			// set new points
			//Vector hitp = S->pos(i) + (S->vel(i) * CD->t);//- (S->vel(i) * CD->t);
			//Vector np = hitp + vr * CD->t;//hitp + vr * CD->t;

			Vector hitp = S->pos(i) - (S->vel(i) * CD->t);//???????????????
			Vector x = hitp + vr * CD->t;//?

			/*Vector hitp = S->vel(i) - vn*norm;
			Vector np = hitp * CD->t + S->pos(i);*/
			S->set_pos(i, x);

			// set reflective velocity if hit
			S->set_vel(i, vr);
			
			//dt = ;


			//递归
			while (surf->hit(S->pos(i), S->vel(i), CD->t, *CD))
			{
				Vector v = S->vel(i);
				Vector norm = CD->tri->N();
				vn = norm * S->vel(i);
				vp = S->vel(i) - norm * vn;
				vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

				Vector hitp = S->pos(i) - (S->vel(i) * CD->t);
				Vector x = hitp + vr * CD->t;
				S->set_pos(i, x);

				S->set_vel(i, vr);

			}
		}
	}
}
//===================================general collision================================================


//===============================================cgwxc=============================================
void pba::ElasticRBDCollisionHandler::handle_collisions(const double dt, RigidBodyState& S)
{
	double vn;
	Vector vp, vr;
	bool isHit = false;

	//if (!usetree)
	
	pba::CollisionData CDLarg{ -dt, nullptr, false, false, false, 0 }; //struct
//#pragma omp parallel for
	for (int i = 0; i < S->nb(); i++)
	{
		pba::CollisionData CD{ dt,nullptr, nullptr, false, false, false, i };// = new pba::CollisionData;
		if (surf->hit(S, i, dt, CD))//collisionSurface
		{
			isHit = true;
			// find the largest backwards T (tc) for all particles and triangles
			if (CD.t > CDLarg.t)
			{
				CDLarg = CD;
			}
			if (isHit && !CDLarg.tri)
			{
				int asdasd = 0;
			}
		}
	}

	if (isHit)
	{
		Vector norm = CDLarg.tri->N();
		double tmax = CDLarg.t;

		// 1. update rotation matrix
		Matrix u = rotation(S->angular_velocity, S->angular_velocity.magnitude() * tmax) * S->angular_rotation;

		// Solve for A
		Vector q = inverse(S->inertia_moment()) * (S->get_vector_attr("r", CDLarg.hit_index) ^ norm);
		double p0 = 2 * S->linear_velocity * norm;
		double p1 = S->angular_velocity * S->inertia_moment() * q;
		double p2 = q * S->inertia_moment() * S->angular_velocity;
		double p3 = 1 / S->totalmass();
		double p4 = q * S->inertia_moment() * q;
		double A = -(p0 + p1 + p2) / (p3 + p4);

		if (A == 0)
			return;

		// 2. update center of velocity
		Vector vr = S->linear_velocity + ((A * norm) / S->totalmass());

		// 3. update angular velocity
		Vector wr = S->angular_velocity + q * A;

		// 4. update center of mass
		Vector x = S->center_of_mass - S->linear_velocity * tmax + vr * tmax;

		// 5. update rotation matrix
		Matrix r = rotation(wr, -wr.magnitude() * tmax) * u;

		S->linear_velocity = vr;
		S->angular_velocity = wr;
		S->center_of_mass = x;
		S->angular_rotation = r;

		// 6. Update position and velocity
		for (int i = 0; i < S->nb(); i++)
		{
			Vector pos = S->center_of_mass + S->angular_rotation * S->get_vector_attr("p", i);
			S->set_pos(i, pos);

			Vector r = S->angular_rotation * S->get_vector_attr("p", i);
			S->set_attr("r", i, r);

			Vector _u = S->angular_velocity ^ r;
			Vector vel = S->linear_velocity + _u;
			S->set_vel(i, vel);
		}

		//handle_collisions(tmax, S);
	}
	
}
//===============================================cgwxc=============================================

//=================================================SBD=====================================================
void pba::ElasticSBDCollisionHandler::handle_collisions(const double dt, SoftBodyState & s)
{
	/*double t;
	CollisionTriangle tri;
	CollisionSphere sph;
	bool status;
	bool hit_tri;
	bool hit_sph;
	size_t hit_index;*/
	pba::CollisionData* CD = new pba::CollisionData;//????????????????????????
	double vn;//
	Vector vp, vr;//
	for (int i = 0; i < s->nb(); i++)
	{
		CD->hit_index = i;
		if (surf->hit(s, i, dt, *CD))//进入下一层，triangle
		{
			//update velocity and positions
			Vector v = s->vel(i);
			Vector norm = CD->tri->N();
			vn = norm * s->vel(i);
			vp = s->vel(i) - norm * vn;
			vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);//For Elastic collision

			// set new points

			Vector hitp = s->pos(i) - (s->vel(i) * CD->t);
			Vector x = hitp + vr * CD->t;

			s->set_pos(i, x);

			// set reflective velocity if hit
			s->set_vel(i, vr);


			//递归
			while (surf->hit(s->pos(i), s->vel(i), CD->t, *CD))
			{
				Vector v = s->vel(i);
				Vector norm = CD->tri->N();
				vn = norm * s->vel(i);
				vp = s->vel(i) - norm * vn;
				vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

				Vector hitp = s->pos(i) - (s->vel(i) * CD->t);
				Vector x = hitp + vr * CD->t;
				s->set_pos(i, x);

				s->set_vel(i, vr);

			}
		}
	}
}

//=================================================SBD=====================================================