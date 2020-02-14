//Base class for Geometric Integration

#include "GISolver.h"

using namespace pba;



pba::GISolver pba::CreateGISolverSubstep(pba::GISolver& s, int nbsteps)
{
	return GISolver(new GISolverSubstep(s, nbsteps));
}

pba::GISolver pba::CreateGISolverFourthOrder(pba::GISolver& s)
{
	return GISolver(new GISolverFourthOrder(s));
}

pba::GISolver pba::CreateGISolverSixthOrder(pba::GISolver& s)
{
	return GISolver(new GISolverSixthOrder(s));
}

pba::GISolver pba::CreateLeapFrogSolver( pba::GISolver& A,  pba::GISolver& B )
{
	return pba::GISolver(new pba::LeapFrogSolver(A,B));
}

pba::GISolver pba::CreateForwardEulerSolver( pba::GISolver& A, pba::GISolver& B )
{
	return GISolver(new ForwardEulerSolver(A, B));
}

pba::GISolver pba::CreateBlanesMoanSolver( pba::GISolver& A, pba::GISolver& B )
{
	return GISolver(new BlanesMoanSolver(A, B));
}
