#include "CollisionHandler.h"

void pba::CollisionHandler::set_collision_surface(CollisionSurface& c)
{
	surf = c;
}

//void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& s)
//{
//	surf->hit(s->pos(), s->vel(), s->time(), dt);
//}

//Handler中有Surface的对象,Surface中有Triangle的对象
void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& S)//Explicit中的advance的solve handler-surface-triangle;dt = delta t
{
	//pba::CollisionData* CD = new pba::CollisionData;
	//double vn;//
	//Vector vp, vr;//
	//for (int i = 0; i < S->nb(); i++)
	//{
	//	CD->hit_index = i;
	//	if (surf->hit(S->pos(i), S->vel(i), dt, *CD))//进入下一层，triangle
	//	{
	//		//update velocity and positions
	//		Vector v = S->vel(i);
	//		Vector norm = CD->tri->N();
	//		vn = norm * S->vel(i);
	//		vp = S->vel(i) - norm * vn;
	//		vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);//For Elastic collision

	//		// set new points
	//		//Vector hitp = S->pos(i) + (S->vel(i) * CD->t);//- (S->vel(i) * CD->t);

	//		Vector hitp = S->pos(i) - (S->vel(i) * CD->t);//每次先移动再检测碰撞有没有发生，有的话倒退回去
	//		Vector x = hitp + vr * CD->t;//碰撞位置加上位移得到新的位置

	//		/*Vector hitp = S->vel(i) - vn*norm;
	//		Vector np = hitp * CD->t + S->pos(i);*/
	//		S->set_pos(i, x);

	//		// set reflective velocity if hit
	//		S->set_vel(i, vr);
	//		
	//		//dt = ;

	//		//递归
	//		while (surf->hit(S->pos(i), S->vel(i), CD->t, *CD))
	//		{
	//			Vector v = S->vel(i);
	//			Vector norm = CD->tri->N();
	//			vn = norm * S->vel(i);
	//			vp = S->vel(i) - norm * vn;
	//			vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

	//			Vector hitp = S->pos(i) - (S->vel(i) * CD->t);
	//			Vector x = hitp + vr * CD->t;
	//			S->set_pos(i, x);

	//			S->set_vel(i, vr);

	//		}
	//	}
	//}
	double vn;
	Vector vp, vr;
	
	for (int i = 0; i < S->nb(); i++)
	{
		pba::CollisionData CD{ dt, nullptr, false, false, false, 0 };// = new pba::CollisionData;
		while (surf->hit(S->pos(i), S->vel(i), CD.t, CD))
		{
			Vector v = S->vel(i);
			Vector norm = CD.tri->N();
			vn = norm * S->vel(i);
			vp = S->vel(i) - norm * vn;
			vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

			// set new point
			Vector xc = S->pos(i) - (S->vel(i) * CD.t);
			Vector x = xc + vr * CD.t;
			S->set_pos(i, x);

			// set reflective velocity
			S->set_vel(i, vr);
		}
	}

}