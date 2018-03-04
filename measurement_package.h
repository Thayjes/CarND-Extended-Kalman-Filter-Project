//
//  measurement_package.h
//  ExtendedKF
//
//  Created by Thayjes Srivas on 3/3/18.
//

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
    long long timestamp_;
    
    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;
    
    Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */

