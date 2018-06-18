#include "mwoibn/eigen_utils/eigen_utils.h"
#include <functional>

Eigen::Matrix<bool,Eigen::Dynamic,1> mwoibn::eigen_utils::flip(Eigen::Matrix<bool,Eigen::Dynamic,1> vector){

        for(int i = 0; i < vector.size(); i++)
                vector[i] = !vector[i];

        return vector;
}

void mwoibn::eigen_utils::limit2PI(double ref, double& st){
        if(st - ref > 1.57079632) {
//    std::cout << "\t" << ref << "\t" << st << "\t";

                st -= 3.14159265;
//  std::cout << st << std::endl;
                limit2PI(ref, st);
        }
        else if (ref - st > 1.57079632) {
//    std::cout << "\t" << ref << "\t" << st << "\t";
                st += 3.14159265;
//    std::cout << st << std::endl;
                limit2PI(ref, st);
        }
}

void mwoibn::eigen_utils::limitPI(double ref, double& st){
        if(st - ref > 0.78539816) {
//    std::cout << "\t" << ref << "\t" << st << "\t";

                st -= 1.57079632;
//  std::cout << st << std::endl;
                limitPI(ref, st);
        }
        else if (ref - st > 0.78539816) {
//    std::cout << "\t" << ref << "\t" << st << "\t";
                st += 1.57079632;
//    std::cout << st << std::endl;
                limitPI(ref, st);
        }
}

// std::size_t mwoibn::eigen_utils::hash(Eigen::Matrix<bool,Eigen::Dynamic,1> vec){
//
//         std::vector<bool> std_vec(vec.data(), vec.data() + vec.size());
//         return std::hash<std::vector<bool> >{} (std_vec);
//
// }
