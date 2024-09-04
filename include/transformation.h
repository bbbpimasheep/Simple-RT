#pragma once
#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include "global.h"
#include "vector.h"

class Transform {
public:
    // Constructors
    Transform() : matrix(Eigen::Matrix4d::Identity()), matrix_inv(Eigen::Matrix4d::Identity()) {}
    Transform(const Eigen::Matrix4d& _matrix) : matrix(_matrix), matrix_inv(_matrix.inverse()) {}
    Transform(const Eigen::Matrix4d& _matrix, const Eigen::Matrix4d& _matrix_inv) 
     : matrix(_matrix), matrix_inv(_matrix_inv) {}

    // Methods
    Vector3 Apply(const Eigen::Vector4d vec) const {
        auto multiplied_vec = matrix * vec;
        if (vec.w() == 0) 
            return Vector3(multiplied_vec.x(), 
                           multiplied_vec.y(), 
                           multiplied_vec.z());
        auto w_inv = 1.0 / vec.w();
        return Vector3(multiplied_vec.x() * w_inv, 
                       multiplied_vec.y() * w_inv, 
                       multiplied_vec.z() * w_inv);
    }
    Eigen::Matrix4d Matrix() const { return matrix; }
    Eigen::Matrix4d InvMatrix() const { return matrix_inv; }

private:
    // Members
    Eigen::Matrix4d matrix, matrix_inv;
};

// Inline Functions
inline Transform Translate(const Vector3& translation) {
    Eigen::Matrix4d matrix, inversed_matrix;
    matrix << 1, 0, 0, translation.x,
              0, 1, 0, translation.y,
              0, 0, 1, translation.z,
              0, 0, 0, 1;
    inversed_matrix << 1, 0, 0, -translation.x,
                       0, 1, 0, -translation.y,
                       0, 0, 1, -translation.z,
                       0, 0, 0, 1;
    return Transform(matrix, inversed_matrix);
}
inline Transform Scale(const Vector3& scale) {
    Eigen::Matrix4d matrix, inversed_matrix;
    matrix << scale.x, 0, 0, 0,
              0, scale.y, 0, 0,
              0, 0, scale.z, 0,
              0, 0, 0, 1;
    inversed_matrix << 1/scale.x, 0, 0, 0,
                       0, 1/scale.y, 0, 0,
                       0, 0, 1/scale.z, 0,
                       0, 0, 0, 1;
    return Transform(matrix, inversed_matrix);
}
inline Transform RotateY(double thata) {
    auto sin_theta = Sin(DegtoRad(thata));
    auto cos_theta = Cos(DegtoRad(thata));
    Eigen::Matrix4d matrix;
    matrix <<  cos_theta, 0, sin_theta, 0,
              0, 1, 0, 0,
              -sin_theta, 0, cos_theta, 0,
              0, 0, 0, 1;
    return Transform(matrix, matrix.transpose());
}
inline Transform operator*(const Transform& t1, const Transform& t2) {
    return Transform(t1.Matrix() * t2.Matrix(), 
                     t2.InvMatrix() * t1.InvMatrix());
}


#endif // TRANSFORMATION_H