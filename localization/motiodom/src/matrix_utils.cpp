#include "matrix_utils.hpp"

namespace motiodom
{
    Matrix3x3 transpose_matrix(Matrix3x3 matrix)
    {
        return Matrix3x3(
            matrix.m11, matrix.m21, matrix.m31,
            matrix.m12, matrix.m22, matrix.m23,
            matrix.m13, matrix.m23, matrix.m33
        );
    }
    Matrix2x2 transpose_matrix(Matrix2x2 matrix)
    {
        return Matrix2x2(
            matrix.m11, matrix.m21,
            matrix.m12, matrix.m22
        );
    }
    Matrix2x3 transpose_matrix(Matrix3x2 matrix)
    {
        return Matrix2x3(
            matrix.m11, matrix.m21, matrix.m31,
            matrix.m12, matrix.m22, matrix.m32
        );
    }
    Matrix3x2 transpose_matrix(Matrix2x3 matrix)
    {
        return Matrix3x2(
            matrix.m11, matrix.m21,
            matrix.m12, matrix.m22,
            matrix.m13, matrix.m23
        );
    }



    float det_matrix(Matrix3x3 matrix)
    {
        float det = matrix.m11*(matrix.m22*matrix.m33 - matrix.m23*matrix.m32)
            - matrix.m12*(matrix.m21*matrix.m33 - matrix.m23*matrix.m31)
            + matrix.m13*(matrix.m21*matrix.m32 - matrix.m22*matrix.m31);

        return det;
    }
    float det_matrix(Matrix2x2 matrix)
    {
        return (matrix.m11*matrix.m22 - matrix.m12*matrix.m21);
    }



    Matrix2x2 inverse_matrix(Matrix2x2 matrix)
    {
        float coef = 1.0 / det_matrix(matrix);

        return Matrix2x2(
            matrix.m22*coef, -1.0*matrix.m21*coef,
            -1.0*matrix.m12*coef, matrix.m11*coef
        );
    }
    Matrix3x3 inverse_matrix(Matrix3x3 matrix)
    {
        float det_ = det_matrix(matrix);

        if(det_ == 0.0)
        {
            return Matrix3x3(
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);
        }

        auto coaf = cofactor_matrix(matrix);

        auto t_coaf = transpose_matrix(coaf);

        return Matrix3x3(
            coaf.m11/det_, coaf.m12/det_, coaf.m13/det_,
            coaf.m21/det_, coaf.m22/det_, coaf.m23/det_,
            coaf.m31/det_, coaf.m32/det_, coaf.m33/det_
        );
    }
    Matrix3x3 cofactor_matrix(Matrix3x3 matrix)
    {
        Matrix3x3 cofactor(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        Matrix2x2 comp_11(matrix.m22, matrix.m23, matrix.m32, matrix.m33);
        float det_11 = det_matrix(comp_11);
        cofactor.m11 = det_11;

        Matrix2x2 comp_12(matrix.m21, matrix.m23, matrix.m31, matrix.m33);
        float det_12 = det_matrix(comp_12);
        cofactor.m12 = det_12*-1.0;

        Matrix2x2 comp_13(matrix.m21, matrix.m22, matrix.m31, matrix.m32);
        float det_13 = det_matrix(comp_13);
        cofactor.m13 = det_13;

        Matrix2x2 comp_21(matrix.m12, matrix.m13, matrix.m32, matrix.m33);
        float det_21 = det_matrix(comp_21);
        cofactor.m21 = det_21*-1.0;

        Matrix2x2 comp_22(matrix.m11, matrix.m13, matrix.m31, matrix.m33);
        float det_22 = det_matrix(comp_22);
        cofactor.m22 = det_22;

        Matrix2x2 comp_23(matrix.m11, matrix.m12, matrix.m31, matrix.m32);
        float det_23 = det_matrix(comp_23);
        cofactor.m23 = det_23*-1.0;

        Matrix2x2 comp_31(matrix.m12, matrix.m13, matrix.m22, matrix.m23);
        float det_31 = det_matrix(comp_31);
        cofactor.m31 = det_31;

        Matrix2x2 comp_32(matrix.m11, matrix.m13, matrix.m21, matrix.m23);
        float det_32 = det_matrix(comp_32);
        cofactor.m32 = det_32*-1.0;

        Matrix2x2 comp_33(matrix.m11, matrix.m12, matrix.m21, matrix.m22);
        float det_33 = det_matrix(comp_33);
        cofactor.m33 = det_33;

        return cofactor;
    }




    Vector2 multiply(Matrix2x3 a, Vector3 b)
    {
        return Vector2(
            a.m11*b.x + a.m12*b.y + a.m13*b.z,
            a.m21*b.x + a.m22*b.y + a.m23*b.z);
    }
    Matrix3x3 multiply(Matrix3x3 a, Matrix3x3 b)
    {
        return Matrix3x3(
            a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31, a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32, a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33,
            a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31, a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32, a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33,
            a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31, a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32, a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33);
    }
    Matrix3x2 multiply(Matrix3x3 a, Matrix3x2 b)
    {
        return Matrix3x2(
            a.m11*b.m11+a.m12*b.m21+a.m13*b.m31, a.m11*b.m12+a.m12*b.m22+a.m13*b.m32,
            a.m21*b.m11+a.m22*b.m21+a.m23*b.m31, a.m21*b.m12+a.m22*b.m22+a.m23*b.m32,
            a.m31*b.m11+a.m32*b.m21+a.m33*b.m31, a.m31*b.m12+a.m32*b.m22+a.m33*b.m32);
    }
    Vector3 multiply(Matrix3x2 a, Vector2 b)
    {
        return Vector3(
            a.m11*b.x + a.m12*b.y,
            a.m21*b.x + a.m22*b.y,
            a.m31*b.x + a.m32*b.y);
    } 
    Matrix3x2 multiply(Matrix3x2 a, Matrix2x2 b)
    {
        return Matrix3x2(
            a.m11*b.m11 + a.m12*b.m21, a.m11*b.m12 + a.m12*b.m22,
            a.m21*b.m11 + a.m22*b.m21, a.m21*b.m12 + a.m22*b.m22,
            a.m31*b.m11 + a.m32*b.m21, a.m31*b.m12 + a.m32*b.m22);
    }
    Matrix3x3 multiply(Matrix3x2 a, Matrix2x3 b)
    {
        return Matrix3x3(
            a.m11*b.m11+a.m12*b.m21, a.m11*b.m12+a.m12*b.m22, a.m11*b.m13+a.m12*b.m23,
            a.m21*b.m11+a.m22*b.m21, a.m21*b.m12+a.m22*b.m22, a.m21*b.m13+a.m22*b.m23,
            a.m31*b.m11+a.m32*b.m21, a.m31*b.m12+a.m32*b.m22, a.m31*b.m13+a.m32*b.m23);
    }
    Vector3 multiply(Matrix3x3 a, Vector3 b)
    {
        return Vector3(
            a.m11*b.x + a.m12*b.y + a.m13*b.z,
            a.m21*b.x + a.m22*b.y + a.m23*b.z,
            a.m31*b.x + a.m32*b.y + a.m33*b.z
        );
    }



    Matrix2x2 add(Matrix2x2 a, Matrix2x2 b)
    {
        return Matrix2x2(
            a.m11+b.m11, a.m12+b.m12,
            a.m21+b.m21, a.m22+b.m22);
    }
    Matrix3x3 add(Matrix3x3 a, Matrix3x3 b)
    {
        return Matrix3x3(
            a.m11+b.m11, a.m12+b.m12, a.m13+b.m13,
            a.m21+b.m21, a.m22+b.m22, a.m23+b.m23,
            a.m31+b.m31, a.m32+b.m32, a.m33+b.m33);
    }



    Vector3 substract(Vector3 a, Vector3 b)
    {
        return Vector3(
            a.x-b.x,
            a.y-b.y,
            a.z-b.z);
    }
    Matrix3x3 substract(Matrix3x3 a, Matrix3x3 b)
    {
        return Matrix3x3(
            a.m11 - b.m11, a.m12 - b.m12, a.m13 - b.m13,
            a.m21 - b.m21, a.m22 - b.m22, a.m23 - b.m23,
            a.m31 - b.m31, a.m32 - b.m32, a.m33 - b.m33);
    }


    Matrix2x2 to_2x2(Matrix3x3 matrix)
    {
        return Matrix2x2(
            matrix.m11, matrix.m12,
            matrix.m21, matrix.m22);
    }
}