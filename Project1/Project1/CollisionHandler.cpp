#include "CollisionHandler.h"

void pba::CollisionHandler::set_collision_surface(CollisionSurface& c)
{
	surf = c;
}

//void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& s)
//{
//	surf->hit(s->pos(), s->vel(), s->time(), dt);
//}

//Handler����Surface�Ķ���,Surface����Triangle�Ķ���
void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& S)//Explicit�е�advance��solve handler-surface-triangle;dt = delta t
{
	//pba::CollisionData* CD = new pba::CollisionData;
	//double vn;//
	//Vector vp, vr;//
	//for (int i = 0; i < S->nb(); i++)
	//{
	//	CD->hit_index = i;
	//	if (surf->hit(S->pos(i), S->vel(i), dt, *CD))//������һ�㣬triangle
	//	{
	//		//update velocity and positions
	//		Vector v = S->vel(i);
	//		Vector norm = CD->tri->N();
	//		vn = norm * S->vel(i);
	//		vp = S->vel(i) - norm * vn;
	//		vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);//For Elastic collision

	//		// set new points
	//		//Vector hitp = S->pos(i) + (S->vel(i) * CD->t);//- (S->vel(i) * CD->t);

	//		Vector hitp = S->pos(i) - (S->vel(i) * CD->t);//ÿ�����ƶ��ټ����ײ��û�з������еĻ����˻�ȥ
	//		Vector x = hitp + vr * CD->t;//��ײλ�ü���λ�Ƶõ��µ�λ��

	//		/*Vector hitp = S->vel(i) - vn*norm;
	//		Vector np = hitp * CD->t + S->pos(i);*/
	//		S->set_pos(i, x);

	//		// set reflective velocity if hit
	//		S->set_vel(i, vr);
	//		
	//		//dt = ;

	//		//�ݹ�
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