#pragma once

#include <limits>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PolygonMesh Mesh;
typedef Eigen::Matrix4f Mat4;
typedef Eigen::Matrix3f Mat3;
typedef Eigen::Vector3f Vec3;
typedef Eigen::Quaternion<float> Quat;

#define DELTA 0.001
#define MIN_DELTA 1 - DELTA/1.5
#define MAX_DELTA 1 + DELTA/1.5

namespace pcl
{
	namespace registration
	{
		class FPCSRegistration
		{
			struct set4
			{
				int p1_index_;
				int p2_index_;
				int p3_index_;
				int p4_index_;

				set4(int p1, int p2, int p3, int p4)
				{
					p1_index_ = p1;
					p2_index_ = p2;
					p3_index_ = p3;
					p4_index_ = p4;
				}

				const int& operator[] (int index)
				{
					if (index == 0) return p1_index_;
					if (index == 1) return p2_index_;
					if (index == 2) return p3_index_;
					if (index == 3) return p4_index_;
				}
			};

		public:
			FPCSRegistration () : l_ (10),
								  delta_ (0.1),
								  overlap_ (0.8),
								  f_ (1),
								  input_source_ (Cloud::Ptr(new Cloud)),
								  input_target_ (Cloud::Ptr(new Cloud)),
								  source_markers_ (Cloud::Ptr(new Cloud)),
								  target_markers_ (Cloud::Ptr(new Cloud))
			{
			}

			~FPCSRegistration ()
			{
				input_source_->~PointCloud();
				input_target_->~PointCloud();
				source_markers_->~PointCloud();
				target_markers_->~PointCloud();
				copy_source_->~PointCloud();
				copy_target_->~PointCloud();
				basis_->~PointCloud();
				u_->~PointCloud();
			}

			void
			setNumOfIteration (const int l)
			{
				l_ = l;
			}

			void 
			setDelta (const double delta)
			{
				delta_ = delta;
			}

			double
			getDelta ()
			{
				return (delta_);
			}

			void
			setOverlap (const double overlap)
			{
				overlap_ = overlap;
			}

			double
			getOverlap ()
			{
				return (overlap_);
			}

			void 
			setInputSourceCloud (Mesh::Ptr input_source)
			{
				pcl::fromPCLPointCloud2(input_source->cloud, *input_source_);
			}

			void 
			setInputSourceCloud (Cloud::Ptr input_source)
			{
				*input_source_ = *input_source;
			}

			Cloud::Ptr getInputSourceCloud ()
			{
				return (input_source_);
			}

			void 
			setInputTargetCloud (Mesh::Ptr input_target)
			{
				pcl::fromPCLPointCloud2(input_target->cloud, *input_target_);
			}

			void 
			setInputTargetCloud (Cloud::Ptr input_target)
			{
				*input_target_ = *input_target;
			}

			Cloud::Ptr getInputTargetCloud ()
			{
				return (input_target_);
			}

			void 
			setMarkers(Mesh::Ptr source_markers, Mesh::Ptr target_markers)
			{
				pcl::fromPCLPointCloud2(source_markers->cloud, *source_markers_);
				pcl::fromPCLPointCloud2(target_markers->cloud, *target_markers_);
			}

			void 
			setMarkers(Cloud::Ptr source_markers, Cloud::Ptr target_markers)
			{
				*source_markers_ = *source_markers;
				*target_markers_ = *target_markers;
			}

	
			Mat4 
			getTransform ()
			{
				return (getMat4());
			}

			void 
			getTransform (Mat3 &rotation, Vec3 &offset)
			{
				rotation = rotation_;
				offset = offset_;
			}

			int
			align (Cloud::Ptr &output);

			int
			alignWithMarkers (Cloud::Ptr &output);

		private:
			int l_;
			double delta_;
			double overlap_;
			double f_;//fraction = 1, 0.5, 0.25...
			Cloud::Ptr input_source_;
			Cloud::Ptr input_target_;
			Cloud::Ptr copy_source_;
			Cloud::Ptr copy_target_;
			Cloud::Ptr basis_;
			pcl::PointCloud<set4>::Ptr u_;
			Cloud::Ptr source_markers_;
			Cloud::Ptr target_markers_;

			Mat4 matrix_;
			Mat3 rotation_;
			Vec3 offset_;

			int
			findBasis (const double dist);

			int
			findCongruents ();

			int
			correspondencesRegistration (const double dist);

			Quat
			estimateTransform (const int corr_index);

			int
			getIndex (const int pIndex, const std::vector<std::pair<int, int> > set);
	
			Mat4
			getMat4 ();

			Quat
			estimateRotationMatrix (Cloud::Ptr left, Cloud::Ptr right);

			Vec3
			centering (Cloud::Ptr cl);
		};
	}
}
