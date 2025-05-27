#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Simple 2D Vector class
class Vector2 {
public:
    double data[2];
    Vector2();
    double& operator()(int i);
    const double& operator()(int i) const;
};

// Simple 2x2 Matrix class
class Matrix2x2 {
public:
    double data[2][2];
    Matrix2x2();
    void setIdentity();
    double& operator()(int i, int j);
    const double& operator()(int i, int j) const;
    Matrix2x2 transpose() const;
};

// Matrix operations declarations
Matrix2x2 operator*(const Matrix2x2& a, const Matrix2x2& b);
Vector2 operator*(const Matrix2x2& m, const Vector2& v);

class KF {
public:
    static const int iX = 0;
    static const int iV = 1;

    KF(double initialX, double initialV, double accelVariance);
    void predict(double dt, double gyro_rate);
    void update(double measValue, double measVariance);
    double pos() const;
    double vel() const;

private:
    Vector2 m_mean;
    Matrix2x2 m_cov;
    const double m_accelVariance;
};

#endif // KALMAN_FILTER_H
