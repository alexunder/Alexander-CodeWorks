/******************************************************************************

  Copyright (C), 2015-2017, Sino Co., Ltd.

 ******************************************************************************
  File Name     : Quaternion.h
  Version       : Initial Draft
  Author        : lin.lin
  Created       : 2017/8/16
  Last Modified :
  Description   : declaration of class CQuat.
  Function List :
  History       :
  1.Date        : 2017/8/16
    Author      : lin.lin
    Modification: Created file
  note: use command "file" + shortcut key "ctrl + enter" to create this comments templet.
******************************************************************************/
#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include <iostream>
#include <cmath>
using namespace std;
/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

/*----------------------------------------------*
 * routines' implementations                    *
 *----------------------------------------------*/

class CQuat
{
// overload some operators by friend
    friend CQuat operator+ ( const CQuat &quaternion1, const CQuat &quaternion2 );
    friend CQuat operator- ( const CQuat &quaternion1, const CQuat &quaternion2 );
    friend CQuat operator* ( const CQuat &quaternion1, const CQuat &quaternion2 );
    friend CQuat operator* ( const CQuat &quaternion, const double factor );
    friend CQuat operator* ( const double factor, const CQuat &quaternion );
    friend CQuat &operator*=(CQuat &quaternion, const double s);
    /**
     *  @brief Divide a quaternion by a scalar
     *  @param a Quaternion
     *  @param s Scalar
     *  @return Scaled quaternion
     */
    friend CQuat operator/(const CQuat &quaternion, const double s) {
      return operator*(quaternion, 1 / s);
    }
    /**
     *  @brief Divide a quaternion by a scalar, in place
     *  @param a Quaternion to scale
     *  @param s Scalar
     *  @return a
     */
    friend CQuat &operator/=(CQuat &a, const double s) {
      return operator*=(a, 1 / s);
    }
// construction and destruction function
public:
    CQuat();
    CQuat(double q_0, double q_1, double q_2, double q_3);
    CQuat(const double angle, double axis[3]);
    CQuat(double yaw, double pitch, double roll);
    ~CQuat(){};

// quaternion relative algorithms
public:
    // L2 norm of the quaternion
    double Norm();
    // Normalize quaternion
    void   Normalize();
    // Complex conjugate quaternion
    CQuat Conjugate() { return CQuat(a(), -b(), -c(), -d()); }
    // Rotate a vector using this quaternion
    CQuat transform(const CQuat &v) ;
    // Convert a rotation quaternion to its matrix form
    double* quat2Rotation();
    // Create quaternion from matrix
    static CQuat rotation2Quat(double* m);
    // Create a rotation quaterion
    static CQuat Rotation(double theta, double x, double y, double z);
    // Conversion Euler ('ZYX') to Quaternion
    static CQuat angle2quat(double yaw, double pitch, double roll);
    // Convert quaternion to rotation angles
    double* quat2angle();
    // quaternion inversion
    CQuat  Inverse();

/* BEGIN: Added by lin.lin, 2017/9/26 */
    // Lower Programming to improve performance;
    static void   Inverse(const double *q, double *qInv);
    static void   rotation2Quat(const double* m, double* q);
    static void   Rotation(const double theta, const double x, const double y, const double z, double* q);
    static void   angle2quat(const double yaw, const double pitch, const double roll, double* q);
    static void   quat2angle(const double* q, double* arrAngle);
    static void   Print(const double* q);
    static double Norm(const double* q);
    static void   Normalize(double* q);
    static void   Normalize(const double* qSrc, double* qDest);
    static void   quat2Rotation(const double* q, double* R);

    static void QuatAdd(const double* quaternion1, const double* quaternion2, double* q);
    static void QuatSub(const double* quaternion1, const double* quaternion2, double* q);
    static void QuatMul(const double* quaternion1, const double* quaternion2, double* q);
    static void QuatMul(const double factor, const double* quaternion, double* q);
    static void QuatMul(const double* quaternion, const double factor, double* q);
    static void QuatDiv(const double* quaternion, const double factor, double* q);

    static void QuatAdd(double* quaternion);
    static void QuatSub(double* quaternion);
    static void QuatMul(double* q, const double factor);
    static void QuatDiv(double* q, const double factor);
    static void QuatAssign(const double* qSrc, double* qDst);

    static void Conjugate(const double* qSrc, double* qDst) {
        qDst[0] =  qSrc[0];
        qDst[1] = -qSrc[1];
        qDst[2] = -qSrc[2];
        qDst[3] = -qSrc[3];
    }
/* END:   Added by lin.lin, 2017/9/26   PN: */

    // output the value of quaternion
    void   Print();

// overload some operators
public:
    double& operator()(unsigned int i) { return m_q[i]; }
    double &a() { return m_q[0]; } /**< Scalar component */
    double &b() { return m_q[1]; } /**< First complex dimension (i) */
    double &c() { return m_q[2]; } /**< Second complex dimension (j) */
    double &d() { return m_q[3]; } /**< Third complex dimension (k) */
    CQuat &operator+= ( const CQuat &quaternion );
    CQuat &operator-= ( const CQuat &quaternion );
    CQuat &operator= ( const CQuat &quaternion );

private:
    double m_q[4];
};

inline void CQuat::QuatAdd(const double* quaternion1, const double* quaternion2, double* q)
{
    q[0] = quaternion1[0] + quaternion2[0];
    q[1] = quaternion1[1] + quaternion2[1];
    q[2] = quaternion1[2] + quaternion2[2];
    q[3] = quaternion1[3] + quaternion2[3];
}
inline void CQuat::QuatSub(const double* quaternion1, const double* quaternion2, double* q)
{
    q[0] = quaternion1[0] - quaternion2[0];
    q[1] = quaternion1[1] - quaternion2[1];
    q[2] = quaternion1[2] - quaternion2[2];
    q[3] = quaternion1[3] - quaternion2[3];
}
inline void CQuat::QuatMul(const double* quaternion1, const double* quaternion2, double* q)
{
    q[0] = quaternion1[0] * quaternion2[0] - quaternion1[1] * quaternion2[1]
         - quaternion1[2] * quaternion2[2] - quaternion1[3] * quaternion2[3];

    q[1] = quaternion1[0] * quaternion2[1] + quaternion1[1] * quaternion2[0]
         + quaternion1[2] * quaternion2[3] - quaternion1[3] * quaternion2[2];

    q[2] = quaternion1[0] * quaternion2[2] + quaternion1[2] * quaternion2[0]
         + quaternion1[3] * quaternion2[1] - quaternion1[1] * quaternion2[3];

    q[3] = quaternion1[0] * quaternion2[3] + quaternion1[3] * quaternion2[0]
         + quaternion1[1] * quaternion2[2] - quaternion1[2] * quaternion2[1];
}
inline void CQuat::QuatMul(const double factor, const double* quaternion, double* q)
{
    q[0] = factor * quaternion[0];
    q[1] = factor * quaternion[1];
    q[2] = factor * quaternion[2];
    q[3] = factor * quaternion[3];
}
inline void CQuat::QuatMul(const double* quaternion, const double factor, double* q)
{
    q[0] = factor * quaternion[0];
    q[1] = factor * quaternion[1];
    q[2] = factor * quaternion[2];
    q[3] = factor * quaternion[3];
}
inline void CQuat::QuatDiv(const double* quaternion, const double factor, double* q)
{
    q[0] = quaternion[0] / factor;
    q[1] = quaternion[1] / factor;
    q[2] = quaternion[2] / factor;
    q[3] = quaternion[3] / factor;
}
inline void CQuat::QuatSub(double* quaternion)
{
    quaternion[0] --;
    quaternion[1] --;
    quaternion[2] --;
    quaternion[3] --;
}
inline void CQuat::QuatMul(double* q, const double factor)
{
    q[0] *= factor;
    q[1] *= factor;
    q[2] *= factor;
    q[3] *= factor;
}

inline void CQuat::QuatDiv(double* q, const double factor)
{
    q[0] /= factor;
    q[1] /= factor;
    q[2] /= factor;
    q[3] /= factor;
}
inline void CQuat::QuatAssign(const double* qSrc, double* qDst)
{
    qDst[0] = qSrc[0];
    qDst[1] = qSrc[1];
    qDst[2] = qSrc[2];
    qDst[3] = qSrc[3];
}

inline double CQuat::Norm(const double* q)
{
    return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}
inline void CQuat::Normalize(double* q)
{
    //const double dNorm = Norm(q);
    const double dNorm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= dNorm;
    q[1] /= dNorm;
    q[2] /= dNorm;
    q[3] /= dNorm;
}

inline void CQuat::Normalize(const double* qSrc, double* qDest)
{
    //const double dNorm = Norm(qSrc);
    const double dNorm = sqrt(qSrc[0] * qSrc[0] + qSrc[1] * qSrc[1] + qSrc[2] * qSrc[2] + qSrc[3] * qSrc[3]);
    qDest[0] = qSrc[0] / dNorm;
    qDest[1] = qSrc[1] / dNorm;
    qDest[2] = qSrc[2] / dNorm;
    qDest[3] = qSrc[3] / dNorm;
}
inline void CQuat::Inverse(const double *q, double *qInv)
{
    double dNorm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    double norm_inv = 1.0 / dNorm2;
    qInv[0] =  norm_inv*q[0];
    qInv[1] = -norm_inv*q[1];
    qInv[2] = -norm_inv*q[2];
    qInv[3] = -norm_inv*q[3];
}
inline void CQuat::quat2Rotation(const double* q, double* R)
{
    double a = q[0];
    double b = q[1];
    double c = q[2];
    double d = q[3];

    double aa = a * a;
    double bb = b * b;
    double cc = c * c;
    double dd = d * d;

    R[0] = aa + bb - cc - dd;
    R[3] = 2 * b * c + 2 * a * d;
    R[6] = 2 * b * d - 2 * a * c;

    R[1] = 2 * b * c - 2 * a * d;
    R[4] = aa - bb + cc - dd;
    R[7] = 2 * c * d + 2 * a * b;

    R[2] = 2 * b * d + 2 * a * c;
    R[5] = 2 * c * d - 2 * a * b;
    R[8] = aa - bb - cc + dd;
}
inline void CQuat::Rotation(const double theta, const double x, const double y, const double z, double* q)
{
    const double dSine   = sin(theta/2);
    const double dCosine = cos(theta/2);
    q[0] = dCosine;
    q[1] = dSine * x;
    q[2] = dSine * y;
    q[3] = dSine * z;
}
inline void CQuat::angle2quat(const double yaw, const double pitch, const double roll, double* q)
{
    double y = yaw   / 2.0;
    double p = pitch / 2.0;
    double r = roll  / 2.0;

    double siny = sin(y);
    double sinp = sin(p);
    double sinr = sin(r);
    double cosy = cos(y);
    double cosp = cos(p);
    double cosr = cos(r);
    q[0] = cosy * cosp * cosr + siny * sinp * sinr;
    q[1] = cosy * cosp * sinr - siny * sinp * cosr;
    q[2] = cosy * sinp * cosr + siny * cosp * sinr;
    q[3] = siny * cosp * cosr - cosy * sinp * sinr;
}
inline void CQuat::quat2angle(const double* qSrc, double* arrAngle)
{
    double q[4] = {0};
    Normalize(qSrc, q);
    double &w = q[0];
    double &x = q[1];
    double &y = q[2];
    double &z = q[3];

    double r11, r12, r21, r31, r32;
    r11 = 2*(x*y + w*z);
    r12 = w*w + x*x - y*y - z*z;
    r21 = -2 * (x*z - w*y);
    r31 = 2 * (y*z + w*x);
    r32 = w*w - x*x - y*y + z*z;

    // find angles for rotations about X, Y, and Z axes
    arrAngle[0] = atan2( r11, r12 );
    arrAngle[1] = asin( r21 );
    arrAngle[2] = atan2( r31, r32 );
}
inline void CQuat::Print(const double* q)
{
    cout << q[0] << "\t";
    cout << q[1] << "\t";
    cout << q[2] << "\t";
    cout << q[3] << endl;
}

#endif
