#include "FPCSRegistration.h"

int 
pcl::registration::FPCSRegistration::alignWithMarkers (Cloud::Ptr &output)
{
	//downsampling();
	double max_dist = 0;
	for (int i = 0; i < input_source_->size()-6; i += 5)
	{
		for (int j = i+1; j < input_source_->size(); ++j)
		{
			double d = pcl::euclideanDistance(input_source_->points[i], input_source_->points[j]);
			if (d > max_dist)
			{
				max_dist = d;
			}
		}
	}
	int h = 0;
	double err = 0;

	std::vector<int> inds_source, inds_target;
	Cloud::Ptr source_corr(new Cloud), target_corr(new Cloud);
	pcl::KdTreeFLANN<Point> tree;
	tree.setInputCloud(input_source_);
	for (int i = 0; i < source_markers_->size(); ++i)
	{
		std::vector<int> inds;
		std::vector<float> dists;
		tree.radiusSearch(source_markers_->points[i], 0.1, inds, dists);
		for (int j = 0; j < inds.size(); ++j)
		{
			inds_source.push_back(inds[j]);
			source_corr->push_back(input_source_->points[inds[j]]);
		}
	}

	tree.setInputCloud(input_target_);
	for (int i = 0; i < target_markers_->size(); ++i)
	{
		std::vector<int> inds;
		std::vector<float> dists;
		tree.radiusSearch(target_markers_->points[i], 0.1, inds, dists);
		for (int j = 0; j < inds.size(); ++j)
		{
			inds_target.push_back(inds[j]);
			target_corr->push_back(input_target_->points[inds[j]]);
		}
	}
	
	for (int i = 0; i < l_; ++i)
	{
		u_ = pcl::PointCloud<set4>::Ptr(new pcl::PointCloud<set4>);
		f_ = 1;
		bool next = false;

		while (!correspondencesRegistration(max_dist * overlap_ * f_))
		{
			std::cout << "Registration stopped. Iteration #" << i << std::endl;
			if (f_ < 0.25)
			{
				f_ = 1;
				next = true;
				break;
			}
			f_ /= 2;
		}
		if (next) continue;

		Vec3 oP(0,0,0), oQ(0,0,0);

		//std::cout << "U.size: " << U->size() << std::endl;
		for (int curr = 0; curr < u_->size(); ++curr)
		{
			copy_source_ = Cloud::Ptr(new Cloud);
			*copy_source_ = *input_source_;
			copy_target_ = Cloud::Ptr(new Cloud);
			*copy_target_ = *input_target_;
			
			oP = centering(copy_source_);
			oQ = centering(copy_target_);
			
			int hi = 0;
			double errori = 0;
			Cloud::Ptr sij(new Cloud);
			Quat q = estimateTransform(curr);
			Vec3 o(0,0,0);
			
			pcl::transformPointCloud(*copy_source_, *copy_source_, o, q);

			for (int j = 0; j < inds_source.size(); ++j)
			{
				for (int j1 = 0; j1 < inds_target.size(); ++j1)
				{
					double dd = pcl::euclideanDistance(copy_source_->points[inds_source[j]], copy_target_->points[inds_target[j1]]);
					if (dd <= delta_)
					{
						errori += dd;
						hi++;
						sij->push_back(copy_source_->points[inds_source[j]]);
						break;
					}
				}
			}
			if (hi > h)
			{
				h = hi;
				err = errori / hi;
				rotation_ = q.matrix();
				offset_ = -oP + oQ;
				
				Cloud::Ptr Uij(new Cloud);
				Uij->push_back(copy_target_->points[u_->points[curr][0]]);
				Uij->push_back(copy_target_->points[u_->points[curr][1]]);
				Uij->push_back(copy_target_->points[u_->points[curr][2]]);
				Uij->push_back(copy_target_->points[u_->points[curr][3]]);
				
				//std::cout << "Best: " << h << std::endl;
				
				Uij->~PointCloud();
				if (h > inds_source.size() * 0.95)
				{
					matrix_ = getMat4();
	
					pcl::transformPointCloud(*input_source_, *output, matrix_);
	
					std::cout << "Alignment completed! Transformation is:" << std::endl << getTransform() << std::endl << "Error: " << err << std::endl;
				}
			}
		}
	}
	matrix_ = getMat4();
	
	pcl::transformPointCloud(*input_source_, *output, matrix_);
	
	std::cout << "Alignment completed! Transformation is:" << std::endl << getTransform() << std::endl << "Error: " << err << std::endl;

	return 1;
}

int
pcl::registration::FPCSRegistration::align (Cloud::Ptr &output)
{
	double max_dist = 0;
	for (int i = 0; i < input_source_->size()-6; i += 5)
	{
		for (int j = i+1; j < input_source_->size(); ++j)
		{
			double d = pcl::euclideanDistance(input_source_->points[i], input_source_->points[j]);
			if (d > max_dist)
			{
				max_dist = d;
			}
		}
	}
	int h = 0;
	double err = 0;
	for (int i = 0; i < l_; ++i)
	{
		u_ = pcl::PointCloud<set4>::Ptr(new pcl::PointCloud<set4>);
		f_ = 1;
		bool next = false;

		while (!correspondencesRegistration(max_dist * overlap_ * f_))
		{
			std::cout << "Registration stopped. Iteration #" << i << std::endl;
			if (f_ < 0.25)
			{
				f_ = 1;
				next = true;
				break;
			}
			f_ /= 2;
		}
		if (next) continue;

		Vec3 oP(0,0,0), oQ(0,0,0);

		//std::cout << "U.size: " << U->size() << std::endl;
		for (int curr = 0; curr < u_->size(); ++curr)
		{
			copy_source_ = Cloud::Ptr(new Cloud);
			*copy_source_ = *input_source_;
			copy_target_ = Cloud::Ptr(new Cloud);
			*copy_target_ = *input_target_;
			
			oP = centering(copy_source_);
			oQ = centering(copy_target_);
			
			int hi = 0;
			double errori = 0;
			Cloud::Ptr sij(new Cloud);
			Quat q = estimateTransform(curr);
			Vec3 o(0,0,0);
			
			pcl::transformPointCloud(*copy_source_, *copy_source_, o, q);

			for (int j = 0; j < input_source_->size(); ++j)
			{
				for (int j1 = 0; j1 < input_target_->size(); ++j1)
				{
					double dd = pcl::euclideanDistance(input_source_->points[j], input_target_->points[j1]);
					if (dd <= delta_)
					{
						errori += dd;
						hi++;
						sij->push_back(input_source_->points[j]);
						break;
					}
				}
			}
			if (hi > h)
			{
				h = hi;
				err = errori / hi;
				rotation_ = q.matrix();
				offset_ = -oP + oQ;
				
				Cloud::Ptr Uij(new Cloud);
				Uij->push_back(copy_target_->points[u_->points[curr][0]]);
				Uij->push_back(copy_target_->points[u_->points[curr][1]]);
				Uij->push_back(copy_target_->points[u_->points[curr][2]]);
				Uij->push_back(copy_target_->points[u_->points[curr][3]]);
				
				//std::cout << "Best: " << h << std::endl;
				
				Uij->~PointCloud();
			}
		}
	}
	matrix_ = getMat4();
	
	pcl::transformPointCloud(*input_source_, *output, matrix_);
	
	std::cout << "Alignment completed! Transformation is:" << std::endl << getTransform() << std::endl << "Error: " << err << std::endl;

	return 1;
}

int 
pcl::registration::FPCSRegistration::correspondencesRegistration (const double dist)
{
	if (!findBasis(dist))
	{
		std::cout << "basis not found. ";
		return 0;
	}

	try
	{
	if (!findCongruents())
	{
		std::cout << "Congruent sets not found. ";
		return 0;
	}
	}
	catch (std::exception e)
	{
		cout << "Error! " << e.what() << endl;
		return 0;
	}

	return 1;
}

int 
pcl::registration::FPCSRegistration::findCongruents ()
{
	std::cout << "Congruents..." << std::endl;
	
	float e_x = ((basis_->points[2].y-basis_->points[0].y)*(basis_->points[1].x-basis_->points[0].x)*(basis_->points[3].x-basis_->points[2].x) +
				basis_->points[0].x*(basis_->points[3].x-basis_->points[2].x)*(basis_->points[1].y-basis_->points[0].y) -
				basis_->points[2].x*(basis_->points[1].x-basis_->points[0].x)*(basis_->points[3].y-basis_->points[2].y)) / 
				((basis_->points[3].x-basis_->points[2].x)*(basis_->points[1].y-basis_->points[0].y) - 
				(basis_->points[1].x-basis_->points[0].x)*(basis_->points[3].y-basis_->points[2].y)),
		e_y = (e_x - basis_->points[0].x) * (basis_->points[1].y-basis_->points[0].y) / (basis_->points[1].x-basis_->points[0].x) + basis_->points[0].y,
		e_z = (e_x - basis_->points[0].x) * (basis_->points[1].z-basis_->points[0].z) / (basis_->points[1].x-basis_->points[0].x) + basis_->points[0].z;
	
	Point e(e_x, e_y, e_z);

	float d1 = pcl::euclideanDistance(basis_->points[0], basis_->points[1]),
		d2 = pcl::euclideanDistance(basis_->points[2], basis_->points[3]);
	if (d1 == 0 || d2 == 0) return 0;
	float r1 = pcl::euclideanDistance(basis_->points[0], e) / d1,
		  r2 = pcl::euclideanDistance(basis_->points[2], e) / d2;
	if (r1 > 1 || r2 > 1) return 0;

	std::vector<std::pair<int,int> > pairs1, pairs2;

	for (int i1 = 0; i1 < input_target_->size()-1; ++i1)
	{
		for (int i2 = i1+1; i2 < input_target_->size(); ++i2)
		{
			std::pair<int,int> p;
			float dist = pcl::euclideanDistance(input_target_->points[i1], input_target_->points[i2]);
			p.first = i1;
			p.second = i2;
			if(dist >= d1 * MIN_DELTA && dist <= d1 * MAX_DELTA)
			{
				pairs1.push_back(p);
			}
			if(dist >= d2 * MIN_DELTA && dist <= d2 * MAX_DELTA)
			{
				pairs2.push_back(p);
			}
			if (pairs1.size() > input_target_->size() || pairs2.size() > input_target_->size())
			{
				i1 = input_target_->size() - 2;
				break;
			}
		}
	}

	if (pairs1.size() < 1 || pairs2.size() < 1) return 0;

	Cloud::Ptr e1_set(new Cloud);
	std::vector<std::pair<int, int> > e1_inds, e2_inds;
	
	for (int i = 0; i < pairs1.size(); ++i)
	{
		Point ee1(0,0,0),ee2(0,0,0);
		ee1.x = input_target_->points[pairs1[i].first].x + r1*(input_target_->points[pairs1[i].second].x - input_target_->points[pairs1[i].first].x);
		ee1.y = input_target_->points[pairs1[i].first].y + r1*(input_target_->points[pairs1[i].second].y - input_target_->points[pairs1[i].first].y);
		ee1.z = input_target_->points[pairs1[i].first].z + r1*(input_target_->points[pairs1[i].second].z - input_target_->points[pairs1[i].first].z);
		e1_set->push_back(ee1);
		int p_index = e1_set->size()-1;
		std::pair<int, int> p1(p_index, i);
		e1_inds.push_back(p1);

		ee2.x = input_target_->points[pairs1[i].first].x + r2*(input_target_->points[pairs1[i].second].x - input_target_->points[pairs1[i].first].x);
		ee2.y = input_target_->points[pairs1[i].first].y + r2*(input_target_->points[pairs1[i].second].y - input_target_->points[pairs1[i].first].y);
		ee2.z = input_target_->points[pairs1[i].first].z + r2*(input_target_->points[pairs1[i].second].z - input_target_->points[pairs1[i].first].z);
		e1_set->push_back(ee2);
		p_index = e1_set->size()-1;
		std::pair<int, int> p2(p_index, i);
		e1_inds.push_back(p2);
	}

	pcl::KdTreeFLANN<Point> kd_tree;
	kd_tree.setInputCloud(e1_set);

	Cloud::Ptr e2_set(new Cloud);

	for (int i = 0; i < pairs2.size(); ++i)
	{
		Point ee1(0,0,0),ee2(0,0,0);
		ee1.x = input_target_->points[pairs2[i].first].x + r1*(input_target_->points[pairs2[i].second].x - input_target_->points[pairs2[i].first].x);
		ee1.y = input_target_->points[pairs2[i].first].y + r1*(input_target_->points[pairs2[i].second].y - input_target_->points[pairs2[i].first].y);
		ee1.z = input_target_->points[pairs2[i].first].z + r1*(input_target_->points[pairs2[i].second].z - input_target_->points[pairs2[i].first].z);
		e2_set->push_back(ee1);
		int p_index = e2_set->size() - 1;
		std::pair<int, int> p1(p_index, i);
		e2_inds.push_back(p1);

		ee2.x = input_target_->points[pairs2[i].first].x + r2*(input_target_->points[pairs2[i].second].x - input_target_->points[pairs2[i].first].x);
		ee2.y = input_target_->points[pairs2[i].first].y + r2*(input_target_->points[pairs2[i].second].y - input_target_->points[pairs2[i].first].y);
		ee2.z = input_target_->points[pairs2[i].first].z + r2*(input_target_->points[pairs2[i].second].z - input_target_->points[pairs2[i].first].z);
		e2_set->push_back(ee2);
		p_index = e2_set->size() - 1;
		std::pair<int, int> p2(p_index, i);
		e2_inds.push_back(p2);
	}

	for (int i = 0; i < e2_set->size(); ++i)
	{
		std::vector<int> inds;
		std::vector<float> dists;
		Point e2 = e2_set->points[i];
		kd_tree.radiusSearch(*e2_set, i, DELTA/2, inds, dists);
		for (int j = 0; j < inds.size(); ++j)
		{
			set4 Ui(-27,-27,-27,-27);
			int indexx = getIndex(inds[j], e1_inds);
			if (indexx != -1)
			{
				Ui.p1_index_ = pairs1[indexx].first;
				Ui.p2_index_ = pairs1[indexx].second;
				indexx = getIndex(i, e2_inds);
				if (indexx != -1)
				{
					Ui.p3_index_ = pairs2[indexx].first;
					Ui.p4_index_ = pairs2[indexx].second;
					u_->push_back(Ui);
				}
			}
		}
	}

	e1_set->~PointCloud();
	e2_set->~PointCloud();
	e1_inds.~vector();
	e2_inds.~vector();

	if (u_->size() < 1) return 0;

	return 1;
}

int 
pcl::registration::FPCSRegistration::getIndex (const int p_index, const std::vector<std::pair<int, int> > set)
{
	for (int i = 0; i < set.size(); ++i)
	{
		if (set[i].first == p_index)
		{
			return set[i].second;
		}
	}
	return -1;
}

Quat
pcl::registration::FPCSRegistration::estimateTransform (const int corr_index)
{
	Cloud::Ptr Uij(new Cloud);
	Uij->push_back(copy_target_->points[u_->points[corr_index][0]]);
	Uij->push_back(copy_target_->points[u_->points[corr_index][1]]);
	Uij->push_back(copy_target_->points[u_->points[corr_index][2]]);
	Uij->push_back(copy_target_->points[u_->points[corr_index][3]]);
	Quat q = estimateRotationMatrix(basis_, Uij);
	return q;
}

int 
pcl::registration::FPCSRegistration::findBasis (const double dist)
{
	std::cout << "Basis..." << std::endl;
	basis_ = Cloud::Ptr(new Cloud);
	bool next = false;

	int index = (rand()/(double)RAND_MAX)*(input_source_->size()-1);
	basis_->push_back(input_source_->points[index]);
	pcl::KdTreeFLANN<Point> tree;
	tree.setInputCloud(input_source_);
	std::vector<int> is;
	std::vector<float> ds;
	tree.radiusSearch(basis_->points[0], dist*MAX_DELTA, is, ds);
	for (index = 0; index < is.size(); ++index)
	{
		double dd = pcl::euclideanDistance(basis_->points[0], input_source_->points[is[index]]);
		if (dd > dist * MIN_DELTA / 2)
		{
			basis_->push_back(input_source_->points[is[index]]);
			break;
		}
	}
	if (index == is.size()) 
	{
		basis_->~PointCloud();
		std::cout << "Point 2 in ";
		return 0;
	}
	for (index = 0; index < is.size(); ++index)
	{
		double dd2 = pcl::euclideanDistance(basis_->points[1], input_source_->points[is[index]]);
		if (dd2 < dist*MAX_DELTA)
		{
			basis_->push_back(input_source_->points[index]);
			break;
		}
	}
	if (index == is.size()) 
	{
		basis_->~PointCloud();
		std::cout << "Point 3 in ";
		return 0;
	}
	next = false;
	//tree.radiusSearch(basis_->points[2], dist*MAX_DELTA, is, ds);
	for (int i = 0; i < input_source_->size(); ++i)
	{
		if (pcl::euclideanDistance(basis_->points[0], input_source_->points[i]) == 0 || pcl::euclideanDistance(basis_->points[0], input_source_->points[i]) < dist*MIN_DELTA ||
			pcl::euclideanDistance(basis_->points[1], input_source_->points[i]) == 0 || pcl::euclideanDistance(basis_->points[1], input_source_->points[i]) < dist*MIN_DELTA ||
			pcl::euclideanDistance(basis_->points[2], input_source_->points[i]) == 0 || pcl::euclideanDistance(basis_->points[2], input_source_->points[i]) < dist*MIN_DELTA)
		{
			continue;
		}
		float x = input_source_->points[i].x,
		y = input_source_->points[i].y,
		z = input_source_->points[i].z,
		x1 = basis_->points[0].x,
		y1 = basis_->points[0].y,
		z1 = basis_->points[0].z,
		x2 = basis_->points[1].x,
		y2 = basis_->points[1].y,
		z2 = basis_->points[1].z,
		x3 = basis_->points[2].x,
		y3 = basis_->points[2].y,
		z3 = basis_->points[2].z;
		double det = ((x-x1)*(y2-y1)*(z3-z1) + (y-y1)*(z2-z1)*(x3-x1) + (z-z1)*(x2-x1)*(y3-y1)) -
				((z-z1)*(y2-y1)*(x3-x1) + (x-x1)*(z2-z1)*(y3-y1) + (z3-z1)*(y-y1)*(x2-x1));
		
		if (fabs(det) < 0.01)
		{
			basis_->push_back(input_source_->points[i]);
			next = true;
			break;
		}
	}
	if (next) 
	{
		return 1;
	}
	basis_->~PointCloud();
	std::cout << "Point 4 in ";
	return 0;
}

Mat4 
pcl::registration::FPCSRegistration::getMat4 ()
{
	matrix_ = matrix_.Identity();
	for (int ind1 = 0; ind1 < 3; ++ind1)
	{
		for (int ind2 = 0; ind2 < 3; ++ind2)
		{
			matrix_(ind1, ind2) = rotation_(ind1, ind2);
		}
	}
	matrix_(0,3) = offset_.x();matrix_(1,3) = offset_.y();matrix_(2,3) = offset_.z();matrix_(3,3) = 1;
	matrix_(3,0) = 0;matrix_(3,1) = 0;matrix_(3,2) = 0;
	return matrix_;
}

Quat
pcl::registration::FPCSRegistration::estimateRotationMatrix (Cloud::Ptr left, Cloud::Ptr right)
{
	Cloud::Ptr left1(new Cloud), right1(new Cloud);
	*left1 = *left;
	*right1 = *right;
	
	//1.planes rotation

	//left surface normal
	float x1 = left1->points[0].x, y1 = left1->points[0].y, z1 = left1->points[0].z,
		  x2 = left1->points[1].x, y2 = left1->points[1].y, z2 = left1->points[1].z,
		  x3 = left1->points[2].x, y3 = left1->points[2].y, z3 = left1->points[2].z;
	Vec3 n1((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1), (x2-x1)*(z3-z1) - (x3-x1)*(z2-z1), (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1));
	
	Vec3 l1(x1, y1, z1), l2(x2, y2, z2), l3(x3,y3,z3), nl = l2.cross(l1);

	//right surface normal
	x1 = right1->points[0].x; y1 = right1->points[0].y; z1 = right1->points[0].z;
	x2 = right1->points[1].x; y2 = right1->points[1].y; z2 = right1->points[1].z;
	x3 = right1->points[2].x; y3 = right1->points[2].y; z3 = right1->points[2].z;
	
	Vec3 n2((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1), (x2-x1)*(z3-z1) - (x3-x1)*(z2-z1), (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1));
	
	Vec3 r1(x1, y1, z1), r2(x2, y2, z2), r3(x3, y3, z3), nr = r2.cross(r1);
	
	n1 = nl;
	n2 = nr;
	if (n1.isZero() || n2.isZero())
	{
		left1->~PointCloud();
		right1->~PointCloud();
		Quat q(0,0,0,0);
		cout << "bad normals..." << endl;
		return q;
	}

	
	//crossing line for 2 planes (a) and sin & cos for rotation
	Vec3 a = n1.cross(n2);
	a /= a.norm();
	
	n1 /= n1.norm();
	n2 /= n2.norm();

	float sin, cos = n1.dot(n2);
	sin = n1.cross(n2).norm();

	///need cos alpha/2 and sin alpha/2 if cos alpha = n1 dot n2 and sin alpha = n1 cross n2
	cos = sqrt((cos + 1)/2);
	sin = cos == 0 ? 1 : sin/(2*cos);
	
	Quat qa(cos, sin*a.x(), sin*a.y(), sin*a.z());
	
	//applying plane rotation
	std::vector<Vec3> ll;
	for (int i = 0; i < left1->size(); ++i)
	{
		float x1 = left1->points[i].x, y1 = left1->points[i].y, z1 = left1->points[i].z;
		Vec3 l1(x1, y1, z1);

		ll.push_back(qa.matrix() * l1);
		left1->points[i].x = ll[i].x();
		left1->points[i].y = ll[i].y();
		left1->points[i].z = ll[i].z();
	}

	//2.rotation in plane|points rotation
	float C = r1.dot(ll[0]) + r2.dot(ll[1]) + r3.dot(ll[2]), S = (r1.cross(ll[0]) + r2.cross(ll[1]) + r3.cross(ll[2])).dot(n2), 
		osin = S/sqrt(S*S + C*C), ocos = C/sqrt(S*S + C*C);
	osin = ocos == -1 ? 1 : osin/sqrt(2*(1 + ocos));
	ocos = sqrt((ocos + 1)/2);
	
	Quat qp(ocos, osin*n2.x(), osin*n2.y(), osin*n2.z());
	
	//result
	Quat qe = qp*qa;

	ll.~vector();
	right1->~PointCloud();
	left1->~PointCloud();

	return qe;
}

Vec3
pcl::registration::FPCSRegistration::centering (Cloud::Ptr cl)
{
	Point p(0,0,0);
	int number_of_points = cl->size();
	for (int i = 0; i < number_of_points; ++i)
	{
		p.x += cl->points[i].x;
		p.y += cl->points[i].y;
		p.z += cl->points[i].z;
	}
	p.x /= number_of_points;
	p.y /= number_of_points;
	p.z /= number_of_points;
	
	for (int i = 0; i < number_of_points; ++i)
	{
		cl->points[i].x -= p.x;
		cl->points[i].y -= p.y;
		cl->points[i].z -= p.z;
	}
	Vec3 res(p.x, p.y, p.z);

	return res;
}
