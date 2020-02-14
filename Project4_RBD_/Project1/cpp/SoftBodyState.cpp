#include "SoftBodyState.h"

pba::SoftBodyState pba::CreateSoftBody(const std::string & nam)
{
	return SoftBodyState(new SoftBodyStateData(nam));
}

pba::SoftBodyState pba::copy(const SoftBodyState d)
{
	SoftBodyState s = CreateSoftBody();

	//const size_t size = d->nb_area_sets();
	
	size_t si = d->nb_pairs();
	for (int i = 0; i < si; i++)
	{
		s->add_pair(d->get_connected_pair(si)->get_first_node(), d->get_connected_pair(si)->get_second_node());
	}
	si = d->nb_area_sets();
	for (int i = 0; i < si; i++)
	{
		s->add_triangle(d->get_area_set(si)->get_first_node(), d->get_area_set(si)->get_second_node(), d->get_area_set(si)->get_third_node());
	}
	si = d->nb_bendables();
	for (int i = 0; i < si; i++)
	{
		s->add_bend(d->get_bendable(si)->get_first_node(), d->get_bendable(si)->get_second_node(), d->get_bendable(si)->get_third_node(), d->get_bendable(si)->get_fourth_node());
	}

	s->set_edge_threshold(d->get_edge_threshold());
	return s;
}

pba::SoftBodyState pba::GeneratePlanarSoftBody(const pba::Vector & llc, const pba::Vector & urc, int nx, int nz)
{//urc:upper right ...?
	const std::string my_sbd;
	SoftBodyState s = CreateSoftBody(my_sbd);
	s->add((nx+1)*(nz+1));
	double stepx = (urc.X() - llc.X()) / nx;
	double stepz = (urc.Z() - llc.Z()) / nz;
	for (int j=0; j < nz+1; j++) 
	{
		for (int i = 0; i < nx + 1; i++)
		{
			s->set_pos(j * nx +j + i, Vector(llc.X() + stepx*i, urc.Y(), urc.Z()-stepz*j));
		}
	}
	//add pairs
	for (int j = 0; j < nz + 1; j++)
	{
		for (int i = 0; i < nx+1; i++)
		{
			if (i < nx)
			{
				s->add_pair(i + j*nx+j, i + j*nx + j + 1);
				if (j < nz)
				{
					s->add_pair(i + j*nx + j, i + j*nx+j + 1+nx + 1);
				}
			}
			if (j < nz)
			{
				s->add_pair(i + j*nx + j, i + j*nx + nx+1 + j);
				if (i > 0)
				{
					s->add_pair(i + j*nx + j, i + j*nx +j - 1 + nx+1);
				}
			}

		}
	}
	//add triangle
	for (int j = 0; j < nz + 1 ; j++)
	{
		for (int i = 0; i < nx + 1; i++)
		{
			if (i < nx && j < nz)
			{
				s->add_triangle(i + j*nx + j, i + j*nx + j +1, i + j*nx + j + nx + 1);
				s->add_triangle(i + j*nx + j, i + j*nx + j + 1, i + j*nx + j + nx + 1 + 1);
				s->add_triangle(i + j*nx + j, i + j*nx + j + nx + 1, i + j*nx + j + nx + 1 + 1);
				s->add_triangle(i + j*nx + j +1, i + j*nx + j + nx + 1, i + j*nx + j + nx + 1 + 1);
			}
		}
	}

	//add bendables
	for (int j = 0; j < nz; j++)
	{
		for (int i = 0; i < nx; i++)
		{
			s->add_bend(i + j*nx + j, i + j*nx + j + 1, i + j*nx + j + nx + 1, i + j*nx + j + nx + 1 + 1);
		}
	}
	
	
	return s;
}

pba::SoftBodyStateData::SoftBodyStateData(const std::string & nam)
{

}

//pba::SoftBodyStateData::SoftBodyStateData(const SoftBodyStateData & d)
//{
//	size_t s = d.nb_pairs();
//	for (int i = 0; i < s; i++)
//	{
//		connected_pairs.push_back(d.connected_pairs[s]);
//	}
//	s = d.nb_area_sets();
//	for (int i = 0; i < s; i++)
//	{
//		area_sets.push_back(d.area_sets[s]);
//	}
//	s = d.nb_bendables();
//	for (int i = 0; i < s; i++)
//	{
//		bend_pairs.push_back(d.bend_pairs[s]);
//	}
//	set_edge_threshold(d.get_edge_threshold());
//	//...to be continued
//}

void pba::SoftBodyStateData::add_pair(size_t i, size_t j)
{
	double l = (pos(i) - pos(j)).magnitude();
	connected_pairs.push_back(CreateSoftEdge(i, j, l));
	//connected_pairs.push_back(new Softedge(i, j, 0.0));
}

void pba::SoftBodyStateData::add_triangle(size_t i, size_t j, size_t k)
{
	double a = ((pos(i) - pos(j)) ^ (pos(k) - pos(j))).magnitude();
	area_sets.push_back(CreateSoftTriangle(i, j, k, a));
}

void pba::SoftBodyStateData::add_bend(size_t i, size_t j, size_t k, size_t l)
{
	//?????TBD
	Vector e1 = pos(j) - pos(i);
	Vector e2 = pos(k) - pos(i);
	Vector n0 = e1 ^ e2 / (e1 ^ e2).magnitude();
	Vector f1 = pos(j) - pos(l);
	Vector f2 = pos(k) - pos(l);
	Vector n1 = f2 ^ f1 / (f1 ^ f2).magnitude();
	double costheta = -1.0 * n0 * n1;
	Vector h = (pos(k) - pos(j)) / (pos(k) - pos(j)).magnitude();
	double sintheta = -1.0 * h * (n0 ^ n1);
	double theta = atan2(sintheta, costheta);
	bend_pairs.push_back(CreateSoftBendable(i, j, k, l, theta));
}
