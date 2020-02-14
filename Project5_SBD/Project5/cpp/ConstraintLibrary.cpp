#include "ConstraintLibrary.h"
#include "Vector.h"
#include "Constraint.h"
#include "DynamicalState.h"

pba::Constraint pba::CreateParticleOnSphereConstraint(double R, Vector cen, int p)
{
	return pba::Constraint(new pba::ParticleOnSphereConstraint(R, cen, p));
}

double pba::ParticleOnSphereConstraint::compute(DynamicalState& s)
{

	return 0.0;
}

void pba::ParticleOnSphereConstraint::solve(DynamicalState& s, double tol, int maxloop)
{
	
	for (size_t i = 0; i < s->nb(); i++)
	{
		for (size_t ii = 0; ii < maxloop; ii++)
		{
			//int endl = 0;
			Vector testbug = s->pos(i) - P;
			Vector testbug2 = grad(s,i);

			double cx = (s->pos(i) - P) * grad(s, i) - R* R;//grad(s, i)   OR   (s->pos(i) - P) same¡£¡£
			Vector deltax = -1.0 * cx * grad(s, i) / (grad(s, i) * grad(s, i));
			if (deltax.magnitude() > tol)// && endl < maxloop)
			{
				s->set_pos(i, s->pos(i) + deltax);
			}
			//endl++;

			else
			{
				ii = 100000;//break;
			}
		}
		

	}
}

pba::Vector pba::ParticleOnSphereConstraint::grad(DynamicalState& s, int index)
{
	Vector v;
	v = s->pos(index) - P;//pos VS position
	//v.normalize();
	return v;
}

pba::Matrix pba::ParticleOnSphereConstraint::gradgrad(DynamicalState& s, int i, int j)
{
	return Matrix(1.0, 0.0, .0, .0, 1.0, .0, .0, .0, 1.0); 
}


void pba::ParticleOnSphereConstraint::set_id(int d)
{
	id = d;
}


