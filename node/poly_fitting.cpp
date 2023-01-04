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

    double a = 1.0, b = 2.0, c = 1.0;

//    generate quadratic curve
    const int N = 10;
    double dt = 2;
    Eigen::Matrix<double, N * M_DIM, 1> Z = Eigen::Matrix<double, N * M_DIM, 1>::Zero();
    Eigen::Matrix<double, N * M_DIM, N_DIM> H = Eigen::Matrix<double, N * M_DIM, N_DIM>::Zero();
    double meas_noise = 0.001;
    int power1 = 2, power2 = 1, power3 = 0;
    for (int i = 0; i < N / M_DIM; i++) {
        double t = i * dt * M_DIM;
        Eigen::Matrix<double, M_DIM, N_DIM> H_i;
        Eigen::Matrix<double, M_DIM, 1> Z_i;
        for (int j = 0; j < M_DIM; j++) {
            H_i.row(j) << pow(t, power1), pow(t, power2), pow(t, power3);
            Z_i(j) = a * pow(t, power1) + b * pow(t, power2) + c * pow(t, power3) + random_normal(meas_noise);
            t += dt;
        }
        H.block<M_DIM, N_DIM>(i * M_DIM, 0) = H_i;
        Z.block<M_DIM, 1>(i * M_DIM, 0) = Z_i;
    }

//    Kalman Filter
    KalmanFilter<N_DIM, M_DIM> kf(true);
    Eigen::Matrix<double, N_DIM, 1> X0 = Eigen::Matrix<double, N_DIM, 1>::Identity();
    X0 << 1, 0, 0;
    Eigen::Matrix<double, N_DIM, N_DIM> P0 = Eigen::Matrix<double, N_DIM, N_DIM>::Identity();
    kf.setInitState(X0, P0);
    double process_noise = 0.00001;
    Eigen::Matrix<double, N_DIM, N_DIM> R = Eigen::Matrix<double, N_DIM, N_DIM>::Identity() * process_noise;
    Eigen::Matrix<double, M_DIM, M_DIM> Q = Eigen::Matrix<double, M_DIM, M_DIM>::Identity() * meas_noise;

    for (int i = 0; i < N / M_DIM; ++i) {
        Eigen::Matrix<double, N_DIM, N_DIM> A = Eigen::Matrix<double, N_DIM, N_DIM>::Identity();
        kf.predict(A, R);

        Eigen::Matrix<double, M_DIM, N_DIM> H_ = H.block<M_DIM, N_DIM>(i * M_DIM, 0);
        Eigen::Matrix<double, M_DIM, 1> Z_ = Z.block<M_DIM, 1>(i * M_DIM, 0);
        kf.update(H_, Q, Z_);
    }
    std::cout << "KalmanFilter done." << std::endl;
    std::cout << "True state: " << a << ", " << b << ", " << c << std::endl;
    kf.print();
    Eigen::Matrix<double, N_DIM, 1> X = kf.getState();
    Eigen::Matrix<double, N_DIM, 1> X_true;
    X_true << a, b, c;
    double error = (X - X_true).norm();
    std::cout << "Error: " << error << std::endl;

    // Least Square
    Eigen::Matrix<double, N_DIM, N_DIM> HTH_inv = (H.transpose() * H).inverse();
    Eigen::Matrix<double, N_DIM, N * M_DIM> HTH_inv_HT = HTH_inv * H.transpose();
    Eigen::Matrix<double, N_DIM, 1> X_ls = HTH_inv_HT * Z;
    std::cout << "X_ls: " << X_ls.transpose() << std::endl;
    error = (X_ls - X_true).norm();
    std::cout << "Error: " << error << std::endl;
    std::cout << "Least Square done." << std::endl;

    // Observability analysis
    std::cout << "Observability analysis of Kalman Filter: " << std::endl;
    // because A is identity matrix, so we ignore A^n, and the observability matrix is H
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd S = svd.singularValues();
    std::cout << "Singular values: " << S.transpose() << std::endl;
    for (int i = 0; i < S.rows(); ++i) {
        std::cout << "Observability of " << i << "-th state: " << S(i) / S(0) << std::endl;
    }

    return 0;
}

double random_normal(double std) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dis(0, std);
    double number = dis(gen);
    return number;
}
