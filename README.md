## Kalman Filter

This repository contains a simple Kalman Filter implementation in C++. In this code, I compare the Kalman Filter with least squares method to estimate the parameters of a linear time-varying system.

Interesting results can be obtained by modifying the hyper-parameter "M_DIM", which reflects the characteristics of the observability of the system.

## Run

```bash
mkdir build
cd build
cmake ..
make
./poly_fitting
```

### Linear Kalman Filter Notes

#### 1. State Space Model

![State Space Model](figures/1.jpg)

#### 2. Kalman Filter Derivation

![Kalman Filter Derivation](figures/2.jpg)

#### 3. Curve Fitting Example

![Curve Fitting Example](figures/3.jpg)