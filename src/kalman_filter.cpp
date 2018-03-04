#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout; using std::endl;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - H_*x_;
    MatrixXd K = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    x_ = x_ + K*y;
    P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    // Calculate the predicted measurement from the state
    VectorXd h;
    float phi, px, py, vx, vy;
    px = x_(0); py = x_(1); vx = x_(2); vy = x_(3);
    h = VectorXd(3);
    float c1 = px*px + py*py;
    float rho = sqrt(c1);
    if(abs(px) < 0.0001){
        std::cout << "Error while converting vector x_ to polar co-ordinates: Division by zero!" << std::endl;
        px = 0.0001;
    }
    phi = atan2(py, px);
    
    
    float rho_dot;
    if(abs(rho) < 0.0001){
        std::cout << "Error while converting vector x_ to polar co-ordinates: Division by zero!" << std::endl;
        rho = 0.0001;
    }
    rho_dot = (px*vx + py*vy) / rho;
    
    h << rho, phi, rho_dot;
    VectorXd y = z - h;
    while(y(1) > M_PI)
    {
        cout << "Pi = " << M_PI << endl;
        y(1) -= 2*M_PI;
    }
    while(y(1) < M_PI)
    {
        cout << "Pi = " << M_PI << endl;
        y(1) += 2*M_PI;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    x_ = x_ + K*y;
    P_ = (I - K*H_)*P_;
    
}
