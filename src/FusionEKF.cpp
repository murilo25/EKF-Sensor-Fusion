#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  H_radar_ = MatrixXd(3, 4);
  ekf_.x_ = VectorXd(4);
  std::cout << "Test\n";
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  z_radar_ = VectorXd(3);
  z_laser_ = VectorXd(2);

  x_ << 0,
      0,
      0,
      0;
  std::cout << "FusionEKF constructor:\n";
  std::cout << "ekf_.x_ rows: " << x_.rows() << "\nekf_.x_ cols: " << x_.cols() << "\n";

  // state covariance matrix P
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  // state covariance matrix Q
  ekf_.Q_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // set the acceleration noise components
  process_noise_ax = 9;
  process_noise_ay = 9;
}

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


//}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
    * Initialization
    */
    if (!is_initialized_) {
        // initialize states with first measurement
        std::cout << "Initialize states...\n";
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // Initialize state given radar measurements ( range (rho), bearing (phi), range rate )
            // Convert radar from polar to cartesian coordinates and initialize state.
            // radar polar coordinates: (rho, phi) 
            // cartesian coordinates: (x , y)
            // polar to cartesian -> x = rho*cos(phi) ; y = rho*sin(phi)
            std::cout << "Initialize radar...\n";
            x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]),
                  measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]) ,
                  0,
                  0;

            previous_timestamp_ = measurement_pack.timestamp_;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // Initialize state given laser measurements (x,y)
            std::cout << "Initialize laser...\n";

            x_ << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1],
                    0,
                    0;

            previous_timestamp_ = measurement_pack.timestamp_;
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        std::cout << "Initialize states OK\n";
        return;
    }


    /* Prediction */
    std::cout << "Prediction...\n";
    // compute the time elapsed between the current and previous measurements -> dt: expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // Computre new F matrix given delta time elapsed between measurements
    F_ << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // 2. Set the process covariance matrix Q
    Q_ << ((dt * dt * dt * dt) / 4.0) * process_noise_ax, 0, ((dt * dt * dt) / 2.0)* process_noise_ax, 0,
          0, ((dt * dt * dt * dt) / 4.0)* process_noise_ay, 0, ((dt * dt * dt) / 2.0)* process_noise_ay,
          ((dt * dt * dt) / 2.0)* process_noise_ax, 0, dt* dt* process_noise_ax, 0,
          0, ((dt * dt * dt) / 2.0)* process_noise_ay, 0, dt* dt* process_noise_ay;

    // check if R and H are for radar or laser: does not matter for prediction
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);


    ekf_.Predict();

    std::cout << "Prediction OK\n";

    /**
    * Update
    */

    /**
    * TODO:
    * - Use the sensor type to perform the update step.
    * - Update the state and covariance matrices.
    */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //if (0) {    // ignore radar 
        // update H and R
        std::cout << "UpdateEKF...\n";
        H_radar_ = tools.CalculateJacobian(x_);
        z_radar_ << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1],
                    measurement_pack.raw_measurements_[2];
        ekf_.Init(x_, P_, F_, H_radar_, R_radar_, Q_);
        ekf_.UpdateEKF(z_radar_);
        std::cout << "UpdateEKF ok\n";
    } else {
        // update H and R
        std::cout << "Update...\n";
        z_laser_ << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1];
        ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
        ekf_.Update(z_laser_);
        std::cout << "Update ok\n";
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
