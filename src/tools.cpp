#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth){

    VectorXd rmse;
    rmse = VectorXd(4);

    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }
    for (int i = 0; i < estimations.size(); i++)
    {
        VectorXd diff = estimations[i] - ground_truth[i];
        diff = diff.array() * diff.array();   //term by term multiplication
        rmse += diff;
    }
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd J;
    J = MatrixXd(3, 4);
    // pre-compute terms that appears in the jacobian multiple times
    float deno = x_state[0] * x_state[0] + x_state[1] * x_state[1];
    float deno_sqrt = sqrt(deno);
    float deno_sqrt_cube = deno * deno_sqrt;

    if (abs(deno) < 0.001) {
        std::cout << "Division by zero during Jacobian calculation\n";
        //return J;
    }

    // calculate jacobian
    J << x_state[0] / deno_sqrt, x_state[1] / deno_sqrt, 0, 0,
        -x_state[1] / deno, x_state[0] / deno, 0, 0,
        x_state[1] * (x_state[2] * x_state[1] - x_state[3] * x_state[0]) / deno_sqrt_cube, x_state[0] * (x_state[3] * x_state[0] - x_state[2] * x_state[1]) / deno_sqrt_cube, x_state[0]/deno_sqrt, x_state[1] / deno_sqrt;
    return J;
}
