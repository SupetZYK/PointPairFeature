# PPF_Recognition

#### Requirement

- pcl 1.8
- cmake

#### Compile

For successful compilation, you should make some change to pcl headers.

- In "pcl\surface\include\pcl\surface\mls.h", uncomment "#if def _OPENMP #endif" or simply define "_OPENMP" in your project. 

  In the same file, add the following public member function to class MovingLeastSquares.

  ```c++
  /** \brief Get the ptr of normals
  */
  inline NormalCloudPtr
  getNormals() const { return (normals_); }
  ```


- In "pcl\filters\include\pcl\filters\uniform_sampling.h", add the following public member function to class UniformSampling

  ```c++
  IndicesPtr getSelectedIndex()
  {
      IndicesPtr select_index(new std::vector<int>);
      select_index->resize(leaves_.size());
      int cp = 0;
      for (typename boost::unordered_map<size_t, Leaf>::const_iterator it = leaves_.begin(); it != leaves_.end(); ++it)
          select_index->at(cp++) = it->second.idx;
      return select_index;
  };
  ```

You donnot need to recompile your pcl library after these modifications.

### Change Log

