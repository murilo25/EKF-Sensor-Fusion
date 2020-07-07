#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth){

    VectorXd diff;
    VectorXd res;

    diff << 0, 0, 0, 0;
    res << 0, 0, 0, 0;

    for (int i = 0; i < estimations.size(); i++)
    {
        diff = estimations[i] - ground_truth[i];
        res += pow(diff, 2);
    }
    res = sqrt(res / estimations.size());

    return res;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    float deno = x_state[0] * x_state[0] + x_state[1] * x_state[1];
    float deno_sqrt = sqrt(deno);
    float deno_sqrt_cube = pow(deno_sqrt, 3);
    MatrixXd J;
    J = MatrixXd(3, 4);
    J << x_state[0] / deno_sqrt, x_state[1] / deno_sqrt, 0, 0,
        x_state[1] / deno, x_state[0] / deno, 0, 0,
        x_state[1] * (x_state[2] * x_state[1] - x_state[3] * x_state[0]) / deno_sqrt_cube, x_state[0] * (x_state[3] * x_state[0] - x_state[2] * x_state[1]) / deno_sqrt_cube, 0, 0;
    
    return J;
}
