/*
 * =====================================================================================
 *
 *       Filename:  KukaAlgorithms.h
 *
 *    Description:  Some algorithms need to run on Workstation.
 *
 *        Version:  1.0
 *        Created:  8/9/2021 4:36:56 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Alexander Hsu (), chenyu.xu@surgnova.com
 *   Organization:  Surgnova
 *
 * =====================================================================================
 */


#ifndef __H_KUKA_ALGORITHMS
#define __H_KUKA_ALGORITHMS

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

template<class T>
void print_std_vector(std::vector<T> vec, const std::string & name) {
    std::ostringstream line;
    line  << name << ":" << std::endl;
    for (auto iter : vec) {
        line << iter << std::endl;
        line << "-----------" << std::endl;
    }
    line << std::endl;
    std::cout << line.str();
}

void TrajectoyPlanningCartesianMaxtrix(Matrix<double, 6, 1> start,
                                       Matrix<double, 6, 1> end,
                                       unsigned int num,
                                       vector<Matrix<double, 6, 1>> & full_traj,
                                       vector<Matrix<double, 6, 1>> & full_velocity);
#endif
