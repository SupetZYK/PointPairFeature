
#ifndef PCL_FILTERS_SMART_SASMPLING_HPP_
#define PCL_FILTERS_SMART_SASMPLING_HPP_
#include <precomp.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <boost/unordered_map.hpp>

#define ZYK_INSTANTIATE_SMART_SAMPLING(A,B)  template class ZYK_EXPORTS pcl::SmartSampling<A,B>

namespace pcl
{
	template<typename PointT, typename NormalT>
	class SmartSampling : public Filter<PointT>
	{
	typedef typename Filter<PointT>::PointCloud PointCloud;

    using Filter<PointT>::filter_name_;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::getClassName;

	
	typedef typename pcl::PointCloud<NormalT>::ConstPtr NormalsConstPtr;
    public:
		typedef boost::shared_ptr<SmartSampling<PointT, NormalT> > Ptr;
		typedef boost::shared_ptr<const SmartSampling<PointT, NormalT> > ConstPtr;

      /** \brief Empty constructor. */
		SmartSampling() :
        leaves_ (),
        leaf_size_ (Eigen::Vector4f::Zero ()),
        inverse_leaf_size_ (Eigen::Vector4f::Zero ()),
        min_b_ (Eigen::Vector4i::Zero ()),
        max_b_ (Eigen::Vector4i::Zero ()),
        div_b_ (Eigen::Vector4i::Zero ()),
        divb_mul_ (Eigen::Vector4i::Zero ()),
        search_radius_ (0)
      {
		selected_indices_.clear();
        filter_name_ = "SmartSampling";
      }

      /** \brief Destructor. */
		virtual ~SmartSampling()
      {
        leaves_.clear();
      }

      /** \brief Set the 3D grid leaf size.
        * \param radius the 3D grid leaf size
        */
      virtual inline void 
      setRadiusSearch (double radius) 
      { 
        leaf_size_[0] = leaf_size_[1] = leaf_size_[2] = static_cast<float> (radius);
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
        search_radius_ = radius;
			};

	  std::vector<size_t>& getSelectedIndex(){ return selected_indices_; };

	  /** \brief Set the normals computed on the input point cloud
	  * \param[in] normals the normals computed for the input cloud
	  */
	  inline void
		  setAngleThresh(float ang_degree) { angle_thresh=ang_degree; }

	  /** \brief Set the normals computed on the input point cloud
	  * \param[in] normals the normals computed for the input cloud
	  */
	  inline void
		  setNormals(const NormalsConstPtr &normals) { input_normals_ = normals; }

	  /** \brief Get the normals computed on the input point cloud */
	  inline NormalsConstPtr
		  getNormals() const { return (input_normals_); }



    protected:
      /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. */
      struct Leaf
      {
		Leaf()  { index_vector.clear(); };
		std::vector<size_t> index_vector;
		size_t& operator [] (size_t i) { return index_vector[i]; };
		bool empty() { return index_vector.empty(); };
		void push_back(size_t index){ index_vector.push_back(index); };
      };

	  std::vector<size_t> selected_indices_;
	  /** \brief in degrees */
	  float angle_thresh = 1;

	  /** \brief The normals computed at each point in the input cloud */
	  NormalsConstPtr input_normals_;

      /** \brief The 3D grid leaves. */
      boost::unordered_map<size_t, Leaf> leaves_;

      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */ 
      Eigen::Array4f inverse_leaf_size_;

      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

	};

	template<typename PointT, typename NormalT>
	void pcl::SmartSampling<PointT, NormalT>::applyFilter(PointCloud &output)
	{
		// Has the input dataset been set already?
		if (!input_)
		{
			PCL_WARN("[pcl::%s::detectKeypoints] No input points given!\n", getClassName().c_str());
			output.width = output.height = 0;
			output.points.clear();
			return;
		}
		if (!input_normals_)
		{
			PCL_WARN("[pcl::%s::detectKeypoints] No input normals given!\n", getClassName().c_str());
			output.width = output.height = 0;
			output.points.clear();
			return;
		}
		if (input_->size() != input_normals_->size())
		{
			PCL_WARN("[pcl::%s::detectKeypoints] Input points'size not equal to normals'size!\n", getClassName().c_str());
			output.width = output.height = 0;
			output.points.clear();
			return;
		}

		output.height = 1;                    // downsampling breaks the organized structure
		output.is_dense = true;                 // we filter out invalid points

		Eigen::Vector4f min_p, max_p;
		// Get the minimum and maximum dimensions
		pcl::getMinMax3D<PointT>(*input_, min_p, max_p);

		// Compute the minimum and maximum bounding box values
		min_b_[0] = static_cast<int> (floor(min_p[0] * inverse_leaf_size_[0]));
		max_b_[0] = static_cast<int> (floor(max_p[0] * inverse_leaf_size_[0]));
		min_b_[1] = static_cast<int> (floor(min_p[1] * inverse_leaf_size_[1]));
		max_b_[1] = static_cast<int> (floor(max_p[1] * inverse_leaf_size_[1]));
		min_b_[2] = static_cast<int> (floor(min_p[2] * inverse_leaf_size_[2]));
		max_b_[2] = static_cast<int> (floor(max_p[2] * inverse_leaf_size_[2]));

		// Compute the number of divisions needed along all axis
		div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
		div_b_[3] = 0;

		// Clear the leaves
		leaves_.clear();

		// Set up the division multiplier
		divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

		//for counter total points
		size_t count = 0;

		//compute dot thresh
		double thresh = cos(angle_thresh*M_PI / 180);
		// First pass: build a set of leaves with the point index closest to the leaf center
		for (size_t cp = 0; cp < indices_->size(); ++cp)
		{
			if (!input_->is_dense)
				// Check if the point is invalid
			if (!pcl_isfinite(input_->points[(*indices_)[cp]].x) ||
				!pcl_isfinite(input_->points[(*indices_)[cp]].y) ||
				!pcl_isfinite(input_->points[(*indices_)[cp]].z))
				continue;

			Eigen::Vector4i ijk = Eigen::Vector4i::Zero();
			ijk[0] = static_cast<int> (floor(input_->points[(*indices_)[cp]].x * inverse_leaf_size_[0]));
			ijk[1] = static_cast<int> (floor(input_->points[(*indices_)[cp]].y * inverse_leaf_size_[1]));
			ijk[2] = static_cast<int> (floor(input_->points[(*indices_)[cp]].z * inverse_leaf_size_[2]));

			// Compute the leaf index
			int idx = (ijk - min_b_).dot(divb_mul_);
			Leaf& leaf = leaves_[idx];

			//else check the normal difference to every other in this leaf
			bool res = true;
			for (size_t i = 0; i < leaf.index_vector.size(); ++i)
			{
				Eigen::Vector3f np = input_normals_->points[leaf[i]].getNormalVector3fMap();
				Eigen::Vector3f nc = input_normals_->points[(*indices_)[cp]].getNormalVector3fMap();
				if (np.dot(nc)>thresh)
				{
					// Check to see if this point is closer to the leaf center than the previous one we saved
					float diff_cur = (input_->points[(*indices_)[cp]].getVector4fMap() - ijk.cast<float>()).squaredNorm();
					float diff_prev = (input_->points[leaf[i]].getVector4fMap() - ijk.cast<float>()).squaredNorm();

					// If current point is closer, copy its index instead
					if (diff_cur < diff_prev)
						leaf[i] = (*indices_)[cp];

					res = false;
					break;
				}
			}
			if (res){
				leaf.push_back((*indices_)[cp]);
				output.points.push_back(input_->points[(*indices_)[cp]]);
				selected_indices_.push_back((*indices_)[cp]);
				count++;
			}
		}

		//// Second pass: go over all leaves and copy data
		//output.points.resize(count);
		//int cp = 0;

		//for (typename boost::unordered_map<size_t, Leaf>::const_iterator it = leaves_.begin(); it != leaves_.end(); ++it)
		//	output.points[cp++] = input_->points[it->second.idx];
		//output.width = static_cast<uint32_t> (output.points.size());

	}
	

}



#endif