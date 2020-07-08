#include "kalman_filter.h"
#include <math.h> 
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
	
    float z_pred0, z_pred1, z_pred2;

    VectorXd z_pred;
    z_pred = VectorXd(3)

    std::cout << "KF 1\n";
    float Px2 = x_(0)* x_(0);
	float Py2 = x_(1)* x_(1);
	z_pred0 = sqrt(Px2 + Py2);

    std::cout << "KF 1.5\n";
    if ( abs(x_(0)) < 0.001 )
        x_(0) = 0.01;
	z_pred1 = atan2(x_(1),x_(0));
    std::cout << "KF 2\n";
    if ( z_pred1 > 0 ) {
        while ( z_pred1 > (M_PI/2) ) {
            z_pred1 -= M_PI;
        }
    }
    else {
        while ( z_pred1 < (-M_PI / 2) ) {
            z_pred1 += M_PI;
        }
    }
    
    std::cout << "KF 3\n";
	z_pred2 = (x_(0)*x_(2) + x_(1)*x_(3))/z_pred0);

    z_pred << z_pred0,
              z_pred1,
              z_pred2;


    VectorXd y = z - z_pred;
    std::cout << "KF 4\n";
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    std::cout << "KF 5\n";
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
