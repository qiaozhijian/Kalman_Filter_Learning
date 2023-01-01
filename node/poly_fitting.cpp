#include <iostream>
#include <string>
#include <Eigen/Core>
#include "kalman_filter.h"
#include <random>

using namespace std;

double random_normal(double std);

#define N_DIM 3
#define M_DIM 1

int main(int argc, char **argv) {

    double a = 2.0, b = 3.0, c = 4.0;

//    generate quadratic curve
    const int N = 1000;
    double dt = 2;
    double t = dt;
    Eigen::Matrix<double, N, M_DIM> Z;
    Eigen::Matrix<double, N, N_DIM> H;
    double meas_noise = 0.001;
    for (int i = 0; i < N; i++) {
        t = i * dt;
        Z(i, 0) = a * t * t + b * t + c + random_normal(meas_noise);
        H.row(i) << t * t, t, 1;
    }

//    Kalman Filter
    KalmanFilter<N_DIM, M_DIM> kf(false);
    Eigen::Matrix<double, N_DIM, 1> X0 = Eigen::Matrix<double, N_DIM, 1>::Identity();
    Eigen::Matrix<double, N_DIM, N_DIM> P0 = Eigen::Matrix<double, N_DIM, N_DIM>::Identity();
    kf.setInitState(X0, P0);
    Eigen::Matrix<double, N_DIM, N_DIM> R = Eigen::Matrix<double, N_DIM, N_DIM>::Identity() * 0.01;
    Eigen::Matrix<double, M_DIM, M_DIM> Q = Eigen::Matrix<double, M_DIM, M_DIM>::Identity() * meas_noise;

    for (int i = 0; i < N; ++i) {
        Eigen::Matrix<double, N_DIM, N_DIM> A = Eigen::Matrix<double, N_DIM, N_DIM>::Identity();
        kf.predict(A, R);

        Eigen::Matrix<double, M_DIM, N_DIM> H_ = H.row(i);
        Eigen::Matrix<double, M_DIM, M_DIM> Z_ = Z.row(i);
        kf.update(H_, Q, Z_);
    }
    kf.print();
    std::cout << "KalmanFilter done." << std::endl;
    std::cout << "True state: " << a << ", " << b << ", " << c << std::endl;
    Eigen::Matrix<double, N_DIM, 1> X = kf.getState();
    Eigen::Matrix<double, N_DIM, 1> X_true;
    X_true << a, b, c;
    double error = (X - X_true).norm();
    std::cout << "Error: " << error << std::endl;

    // Least Square
    Eigen::Matrix<double, N_DIM, N_DIM> HTH_inv = (H.transpose() * H).inverse();
    Eigen::Matrix<double, N_DIM, N> HTH_inv_HT = HTH_inv * H.transpose();
    Eigen::Matrix<double, N_DIM, 1> X_ls = HTH_inv_HT * Z;
    std::cout << "X_ls: " << X_ls.transpose() << std::endl;
    error = (X_ls - X_true).norm();
    std::cout << "Error: " << error << std::endl;
    std::cout << "Least Square done." << std::endl;

    return 0;
}

double random_normal(double std) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dis(0, std);
    double number = dis(gen);
    return number;
}
