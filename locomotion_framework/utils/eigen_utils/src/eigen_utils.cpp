#include "mwoibn/eigen_utils/eigen_utils.h"


Eigen::Matrix<bool,Eigen::Dynamic,1> mwoibn::eigen_utils::flip(Eigen::Matrix<bool,Eigen::Dynamic,1> vector){

  for(int i = 0; i < vector.size(); i++)
    vector[i] = !vector[i];

  return vector;
}
