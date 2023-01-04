//
// Created by qzj on 2021/12/24.
//

#ifndef CMAKE_TEMPLATE_KalmanFilter_H
#define CMAKE_TEMPLATE_KalmanFilter_H

#include <Eigen/Core>
#include <iostream>
#include <Eigen/Dense>

template<int N_DIM, int M_DIM>
class KalmanFilter {

    typedef Eigen::Matrix<double, N_DIM, 1> VectorNd;
    typedef Eigen::Matrix<double, N_DIM, N_DIM> MatrixNd;
    typedef Eigen::Matrix<double, N_DIM, M_DIM> MatrixNMd;

    typedef Eigen::Matrix<double, M_DIM, 1> VectorMd;
    typedef Eigen::Matrix<double, M_DIM, N_DIM> MatrixMNd;
    typedef Eigen::Matrix<double, M_DIM, M_DIM> MatrixMd;

public:

    KalmanFilter(bool print) {
        print_ = print;
        std::cout << "KalmanFilter init." << std::endl;
    }

    void setInitState(VectorNd X0, MatrixNd P0) {
        X_ = X0;
        P_ = P0;
        std::cout << "setInitState: " << std::endl;
        std::cout << "X = " << X_.transpose() << std::endl;
        std::cout << "Deterrminant of P = " << P_.determinant() << std::endl;
    }

    void predict(MatrixNd &A, MatrixNd &R) {
        X_ = A * X_;
        P_ = A * P_ * A.transpose() + R;
        if (print_) {
            // every time predict, P increases
            std::cout << "After predict, P = \n" << P_ << std::endl;
            std::cout << "After predict, deterrminant of P = " << P_.determinant() << std::endl;
        }
    }

    void update(MatrixMNd &H, MatrixMd &Q, VectorMd &Z) {
        MatrixMd S = H * P_ * H.transpose() + Q;
        MatrixNMd K = P_ * H.transpose() * S.inverse();
        X_ = X_ + K * (Z - H * X_);
        std::cout << "K = \n" << K << std::endl;
        std::cout << "Z - H * X_ = \n" << Z - H * X_ << std::endl;
        P_ = (Eigen::MatrixXd::Identity(N_DIM, N_DIM) - K * H) * P_;
        if (print_) {
            std::cout << "After update, X = " << X_.transpose() << std::endl;
            // every time update, P decreases
            std::cout << "After update, P = \n" << P_ << std::endl;
            std::cout << "After update, Deterrminant of P = " << P_.determinant() << std::endl;
        }
    }

    void print() {
        std::cout << "Kalman filter obtained X_ = " << X_.transpose() << std::endl;
        std::cout << "P_ = " << std::endl << P_ << std::endl;
        std::cout << "Deterrminant of P = " << P_.determinant() << std::endl;
    }

    VectorNd getState() {
        return X_;
    }

private:
    bool print_;
    VectorNd X_;
    MatrixNd P_;
};

#endif //CMAKE_TEMPLATE_KalmanFilter_H
