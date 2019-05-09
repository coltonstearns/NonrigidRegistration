#include <iostream>
#include <vector>
#include <math.h>

// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>

void computeGramKernel(Eigen::MatrixXf source, Eigen::MatrixXf gramKernel, float beta);