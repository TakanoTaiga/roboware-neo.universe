#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

namespace motiodom
{
    struct Vector3
    {
        float x, y, z;

        Vector3(float x_=0.0, float y_=0.0, float z_=0.0):x(x_),y(y_),z(z_){}
    };

    struct Vector2
    {
        float x, y;

        Vector2(float x_, float y_):x(x_),y(y_){}
    };
    
    struct Matrix3x3
    {
        Matrix3x3(float m11_=0.0, float m12_=0.0, float m13_=0.0, float m21_=0.0, float m22_=0.0, float m23_=0.0, float m31_=0.0, float m32_=0.0, float m33_=0.0):
        m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_),
        m31(m31_),m32(m32_),m33(m33_){}

        float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    };

    struct Matrix2x2
    {
        Matrix2x2(float m11_=0.0, float m12_=0.0, float m21_=0.0, float m22_=0.0):
        m11(m11_),m12(m12_),
        m21(m21_),m22(m22_)
        {}

        float m11, m12, m21, m22;
    };

    struct Matrix3x2
    {
        Matrix3x2(float m11_=0.0, float m12_=0.0, float m21_=0.0, float m22_=0.0, float m31_=0.0, float m32_=0.0):
        m11(m11_),m12(m12_),
        m21(m21_),m22(m22_)
        {}
        
        float m11, m12, m21, m22, m31, m32;
    };

    struct Matrix2x3
    {
        Matrix2x3(float m11_=0.0, float m12_=0.0, float m13_=0.0, float m21_=0.0, float m22_=0.0, float m23_=0.0):
        m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_)
        {}

        float m11, m12, m13, m21, m22, m23;
    };

    Matrix3x3 transpose_matrix(Matrix3x3 matrix);
    Matrix2x2 transpose_matrix(Matrix2x2 matrix);
    Matrix2x3 transpose_matrix(Matrix3x2 matrix);
    Matrix3x2 transpose_matrix(Matrix2x3 matrix);

    float det_matrix(Matrix3x3 matrix);
    float det_matrix(Matrix2x2 matrix);

    Matrix2x2 inverse_matrix(Matrix2x2 matrix);
    Matrix3x3 inverse_matrix(Matrix3x3 matrix);
    
    Matrix3x3 cofactor_matrix(Matrix3x3 matrix);

    Vector2 multiply(Matrix2x3 a, Vector3 b);
    Matrix3x3 multiply(Matrix3x3 a, Matrix3x3 b);
    Matrix3x2 multiply(Matrix3x3 a, Matrix3x2 b);
    Vector3 multiply(Matrix3x2 a, Vector2 b);
    Matrix3x2 multiply(Matrix3x2 a, Matrix2x2 b);
    Matrix3x3 multiply(Matrix3x2 a, Matrix2x3 b);
    Vector3 multiply(Matrix3x3 a, Vector3 b);

    Matrix2x2 add(Matrix2x2 a, Matrix2x2 b);
    Matrix3x3 add(Matrix3x3 a, Matrix3x3 b);

    Vector3 substract(Vector3 a, Vector3 b);
    Matrix3x3 substract(Matrix3x3 a, Matrix3x3 b);

    Matrix2x2 to_2x2(Matrix3x3 matrix);

}

#endif 