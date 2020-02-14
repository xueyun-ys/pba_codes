#include "Constraint.h"

pba::MultiConstraint pba::CreateMultiConstraint()
{
	return pba::MultiConstraint(new pba::MultiConstraintBase());
}

void pba::MultiConstraintBase::solve(DynamicalState& s, double tol, int maxloop)
{
	constraints[0]->solve(s, tol, maxloop);
}
