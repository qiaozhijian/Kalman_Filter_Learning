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
    int N = 1000;
    double dt = 2;
    Eigen::VectorXd x(N), y(N);
    double meas_noise = 0.001;
    for (int i = 0; i < N; i++) {
        x(i) = i * dt;
        y(i) = a * x(i) * x(i) + b * x(i) + c + random_normal(meas_noise);
    }

//    Kalman Filter
    KalmanFilter<N_DIM, M_DIM> kf(true);
    Eigen::Matrix<double, N_DIM, 1> X0 = Eigen::Matrix<double, N_DIM, 1>::Identity();
    Eigen::Matrix<double, N_DIM, N_DIM> P0 = Eigen::Matrix<double, N_DIM, N_DIM>::Identity();
    kf.setInitState(X0, P0);
    Eigen::Matrix<double, N_DIM, N_DIM> R = Eigen::Matrix<double, N_DIM, N_DIM>::Identity() * 0.01;
    Eigen::Matrix<double, M_DIM, M_DIM> Q = Eigen::Matrix<double, M_DIM, M_DIM>::Identity() * meas_noise;

    for (int i = 0; i < N; ++i) {
        Eigen::Matrix<double, N_DIM, N_DIM> A = Eigen::Matrix<double, N_DIM, N_DIM>::Identity();
        kf.predict(A, R);

        Eigen::Matrix<double, M_DIM, N_DIM> H;
        H << x(i) * x(i), x(i), 1;
        Eigen::Matrix<double, M_DIM, M_DIM> Z;
        Z << y(i);
        kf.update(H, Q, Z);
    }
    kf.print();
    std::cout << "KalmanFilter done." << std::endl;
    std::cout << "True state: " << a << ", " << b << ", " << c << std::endl;
    Eigen::Matrix<double, N_DIM, 1> X = kf.getState();
    Eigen::Matrix<double, N_DIM, 1> X_true;
    X_true << a, b, c;
    double error = (X - X_true).norm();
    std::cout << "Error: " << error << std::endl;
    return 0;
}

double random_normal(double std) {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, std);
    double number = distribution(generator);
    return number;
}
