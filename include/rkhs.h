#include <iostream>
#include <vector>
#include <math.h>

// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>

Eigen::MatrixXf computeGramKernel(Eigen::MatrixXf source, float beta);