#include "KalmanFilter.h"

// Vector2 implementation
Vector2::Vector2() {
    data[0] = 0;
    data[1] = 0;
}

double& Vector2::operator()(int i) {
    return data[i];
}

const double& Vector2::operator()(int i) const {
    return data[i];
}

// Matrix2x2 implementation
Matrix2x2::Matrix2x2() {
    data[0][0] = 0; data[0][1] = 0;
    data[1][0] = 0; data[1][1] = 0;
}

void Matrix2x2::setIdentity() {
    data[0][0] = 1; data[0][1] = 0;
    data[1][0] = 0; data[1][1] = 1;
}

double& Matrix2x2::operator()(int i, int j) {
    return data[i][j];
}

const double& Matrix2x2::operator()(int i, int j) const {
    return data[i][j];
}

Matrix2x2 Matrix2x2::transpose() const {
    Matrix2x2 result;
    result.data[0][0] = data[0][0];
    result.data[0][1] = data[1][0];
    result.data[1][0] = data[0][1];
    result.data[1][1] = data[1][1];
    return result;
}

// Matrix operations implementations
Matrix2x2 operator*(const Matrix2x2& a, const Matrix2x2& b) {
    Matrix2x2 result;
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            result.data[i][j] = 0;
            for(int k = 0; k < 2; k++) {
                result.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
    return result;
}

Vector2 operator*(const Matrix2x2& m, const Vector2& v) {
    Vector2 result;
    for(int i = 0; i < 2; i++) {
        result.data[i] = 0;
        for(int j = 0; j < 2; j++) {
            result.data[i] += m.data[i][j] * v.data[j];
        }
    }
    return result;
}

// KF implementation
KF::KF(double initialX, double initialV, double accelVariance) : m_accelVariance(accelVariance) {
    m_mean.data[iX] = initialX;
    m_mean.data[iV] = initialV;

    m_cov.setIdentity();
    m_cov.data[0][0] = 1;
    m_cov.data[1][1] = 50;
}

void KF::predict(double dt, double gyro_rate) {
    Matrix2x2 F;
    F.data[0][0] = 1; F.data[0][1] = dt;
    F.data[1][0] = 0; F.data[1][1] = 1;

    m_mean.data[iV] = gyro_rate;

    Vector2 newX = F * m_mean;

    Vector2 G;
    G.data[iX] = 0.5 * dt * dt;
    G.data[iV] = dt;

    Matrix2x2 Q;
    Q.data[0][0] = G.data[0] * G.data[0] * m_accelVariance;
    Q.data[0][1] = G.data[0] * G.data[1] * m_accelVariance;
    Q.data[1][0] = G.data[1] * G.data[0] * m_accelVariance;
    Q.data[1][1] = G.data[1] * G.data[1] * m_accelVariance;

    Matrix2x2 newP = F * m_cov;
    newP = newP * F.transpose();
    
    newP.data[0][0] += Q.data[0][0];
    newP.data[0][1] += Q.data[0][1];
    newP.data[1][0] += Q.data[1][0];
    newP.data[1][1] += Q.data[1][1];

    m_cov = newP;
    m_mean = newX;
}

void KF::update(double measValue, double measVariance) {
    double H[2] = {1, 0};
    double y = measValue - (H[0] * m_mean.data[0] + H[1] * m_mean.data[1]);
    double S = H[0] * (m_cov.data[0][0] * H[0] + m_cov.data[0][1] * H[1]) +
               H[1] * (m_cov.data[1][0] * H[0] + m_cov.data[1][1] * H[1]) + 
               measVariance;
    
    Vector2 K;
    K.data[0] = (m_cov.data[0][0] * H[0] + m_cov.data[0][1] * H[1]) / S;
    K.data[1] = (m_cov.data[1][0] * H[0] + m_cov.data[1][1] * H[1]) / S;
    
    m_mean.data[0] += K.data[0] * y;
    m_mean.data[1] += K.data[1] * y;
    
    Matrix2x2 I;
    I.setIdentity();
    
    Matrix2x2 temp;
    temp.data[0][0] = K.data[0] * H[0];
    temp.data[0][1] = K.data[0] * H[1];
    temp.data[1][0] = K.data[1] * H[0];
    temp.data[1][1] = K.data[1] * H[1];
    
    m_cov.data[0][0] = (1 - temp.data[0][0]) * m_cov.data[0][0] - temp.data[0][1] * m_cov.data[1][0];
    m_cov.data[0][1] = (1 - temp.data[0][0]) * m_cov.data[0][1] - temp.data[0][1] * m_cov.data[1][1];
    m_cov.data[1][0] = -temp.data[1][0] * m_cov.data[0][0] + (1 - temp.data[1][1]) * m_cov.data[1][0];
    m_cov.data[1][1] = -temp.data[1][0] * m_cov.data[0][1] + (1 - temp.data[1][1]) * m_cov.data[1][1];
}

double KF::pos() const {
    return m_mean.data[iX];
}

double KF::vel() const {
    return m_mean.data[iV];
}
