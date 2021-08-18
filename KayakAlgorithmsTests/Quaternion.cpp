/******************************************************************************

  Copyright (C), 2015-2017, Sino Co., Ltd.

 ******************************************************************************
  File Name     : Quaternion.cpp
  Version       : Initial Draft
  Author        : lin.lin
  Created       : 2017/8/16
  Last Modified :
  Description   : implemention of class CQuat
  Function List :
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              CQuat.CQuat
              operator*
              operator*
              operator*
              operator*=
              operator+
              operator-
  History       :
  1.Date        : 2017/8/16
    Author      : lin.lin
    Modification: Created file
  note: use command "file" + shortcut key "ctrl + enter" to create this comments templet.
******************************************************************************/
#include "Quaternion.h"
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

/*****************************************************************************
 Prototype    : CQuat.CQuat
 Description  : Construct a quaterion. Create a quaternion with null rotation
 Input        : None
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat::CQuat()
{
    m_q[0] = 1;
    m_q[1] = m_q[2] = m_q[3] = 0;
}
/*****************************************************************************
 Prototype    : CQuat.CQuat
 Description  : Construct a quaterion
 Input        : double q_0  Scalar parameter
                double q_1  Complex parameters
                double q_2
                double q_3
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat::CQuat( double q_0, double q_1, double q_2, double q_3 )
{
    m_q[0] = q_0;
    m_q[1] = q_1;
    m_q[2] = q_2;
    m_q[3] = q_3;
}
/*****************************************************************************
 Prototype    : CQuat.CQuat
 Description  : Construct a quaterion.
 Input        : const double angle: Angle of rotation, radians
                double axis[3]    : rotation vector
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat::CQuat( const double angle, double axis[3])
{
    const double dSine   = std::sin(angle / 2);
    const double dCosine = std::cos(angle / 2);
    m_q[0] = dCosine;
    m_q[1] = dSine * axis[0];
    m_q[2] = dSine * axis[1];
    m_q[3] = dSine * axis[2];
}

/*****************************************************************************
 Prototype    : CQuat.CQuat
 Description  : Construct a quaterion. Conversion Euler ('ZYX') to Quaternion
 Input        : double yaw    : Euler angles
                double pitch
                double roll
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
  note: see angle2quat.m.
*****************************************************************************/
CQuat::CQuat(double yaw, double pitch, double roll)
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

    m_q[0] = cosy * cosp * cosr + siny * sinp * sinr;
    m_q[1] = cosy * cosp * sinr - siny * sinp * cosr;
    m_q[2] = cosy * sinp * cosr + siny * cosp * sinr;
    m_q[3] = siny * cosp * cosr - cosy * sinp * sinr;
}

/*****************************************************************************
 Prototype    : operator+
 Description  : Add two quaternions (element-wise summation)
 Input        : const CQuat &quaternion1: First quaternion
                const CQuat &quaternion2: Second quaternion
 Output       : None
 Return Value : Sum
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat operator+ ( const CQuat &quaternion1, const CQuat &quaternion2 )
{
    CQuat temp;
    temp.m_q[0] = quaternion1.m_q[0] + quaternion2.m_q[0];
    temp.m_q[1] = quaternion1.m_q[1] + quaternion2.m_q[1];
    temp.m_q[2] = quaternion1.m_q[2] + quaternion2.m_q[2];
    temp.m_q[3] = quaternion1.m_q[3] + quaternion2.m_q[3];
    return temp;
}
/*****************************************************************************
 Prototype    : operator-
 Description  : Substract two quaternions (element-wise subtraction)
 Input        : const CQuat &quaternion1: First quaternion
                const CQuat &quaternion2: Second quaternion
 Output       : None
 Return Value : Subtraction
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat operator- ( const CQuat &quaternion1, const CQuat &quaternion2 )
{
    CQuat temp;
    temp.m_q[0] = quaternion1.m_q[0] - quaternion2.m_q[0];
    temp.m_q[1] = quaternion1.m_q[1] - quaternion2.m_q[1];
    temp.m_q[2] = quaternion1.m_q[2] - quaternion2.m_q[2];
    temp.m_q[3] = quaternion1.m_q[3] - quaternion2.m_q[3];
    return temp;
}

/*****************************************************************************
 Prototype    : operator*
 Description  : Multiply two quaternions
 Input        : const CQuat& quaternion1: Left quaternion
                const CQuat& quaternion2: Right quaternion
 Output       : None
 Return Value : Product of both quaternions
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat operator* ( const CQuat& quaternion1, const CQuat& quaternion2 )
{
    CQuat temp;
    temp.m_q[0] = quaternion1.m_q[0] * quaternion2.m_q[0] - quaternion1.m_q[1] * quaternion2.m_q[1]
                - quaternion1.m_q[2] * quaternion2.m_q[2] - quaternion1.m_q[3] * quaternion2.m_q[3];

    temp.m_q[1] = quaternion1.m_q[0] * quaternion2.m_q[1] + quaternion1.m_q[1] * quaternion2.m_q[0]
                + quaternion1.m_q[2] * quaternion2.m_q[3] - quaternion1.m_q[3] * quaternion2.m_q[2];

    temp.m_q[2] = quaternion1.m_q[0] * quaternion2.m_q[2] + quaternion1.m_q[2] * quaternion2.m_q[0]
                + quaternion1.m_q[3] * quaternion2.m_q[1] - quaternion1.m_q[1] * quaternion2.m_q[3];

    temp.m_q[3] = quaternion1.m_q[0] * quaternion2.m_q[3] + quaternion1.m_q[3] * quaternion2.m_q[0]
                + quaternion1.m_q[1] * quaternion2.m_q[2] - quaternion1.m_q[2] * quaternion2.m_q[1];
    return temp;
}
/*****************************************************************************
 Prototype    : operator*
 Description  : Multiply real number and quaternion
 Input        : const double factor     : real number
                const CQuat &quaternion : quaternion
 Output       : None
 Return Value : Product of real number and quaternion
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat operator* ( const double factor, const CQuat &quaternion )
{
    CQuat temp;
    temp.m_q[0] = factor * quaternion.m_q[0];
    temp.m_q[1] = factor * quaternion.m_q[1];
    temp.m_q[2] = factor * quaternion.m_q[2];
    temp.m_q[3] = factor * quaternion.m_q[3];
    return temp;
}

/*****************************************************************************
 Prototype    : operator*
 Description  : Multiply quaternion and real number
 Input        : const CQuat &quaternion : quaternion
                const double factor     : real number
 Output       : None
 Return Value : Product of quaternion and real number
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat operator* ( const CQuat &quaternion, const double factor )
{
    CQuat temp;
    temp.m_q[0] = factor * quaternion.m_q[0];
    temp.m_q[1] = factor * quaternion.m_q[1];
    temp.m_q[2] = factor * quaternion.m_q[2];
    temp.m_q[3] = factor * quaternion.m_q[3];
    return temp;
}

/*****************************************************************************
 Prototype    : CQuat.operator+=
 Description  : overload operator of self addition
 Input        : const CQuat& quaternion
 Output       : None
 Return Value : CQuat&
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat& CQuat::operator+= ( const CQuat& quaternion )
{
    *this = *this + quaternion;
    return *this;
}
inline void CQuat::QuatAdd(double* quaternion)
{
    quaternion[0] ++;
    quaternion[1] ++;
    quaternion[2] ++;
    quaternion[3] ++;
}

/*****************************************************************************
 Prototype    : CQuat.operator-=
 Description  : overload operator of self subtraction
 Input        : const CQuat& quaternion
 Output       : None
 Return Value : CQuat&
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat& CQuat::operator-= ( const CQuat& quaternion )
{
    *this = *this - quaternion;
    return *this;
}

/*****************************************************************************
 Prototype    : operator*=
 Description  : overload operator of self multiplication
 Input        : CQuat &quaternion
                const double s
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat &operator*= (CQuat &quaternion, const double s)
{
    for (int i = 0; i < 4; i++)
    {
        quaternion(i) *= s;
    }
    return quaternion;
}
/*****************************************************************************
 Prototype    : CQuat.operator=
 Description  : overload operator of assignment
 Input        : const CQuat& quaternion
 Output       : None
 Return Value : CQuat&
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat& CQuat::operator = ( const CQuat& quaternion )
{
    if ( this != &quaternion )
    {
        m_q[0] = quaternion.m_q[0];
        m_q[1] = quaternion.m_q[1];
        m_q[2] = quaternion.m_q[2];
        m_q[3] = quaternion.m_q[3];
    }
    return *this;
}

/*****************************************************************************
 Prototype    : CQuat.Norm
 Description  : L2 norm of the quaternion
 Input        : None
 Output       : None
 Return Value : double
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
double CQuat::Norm()
{
    return std::sqrt(m_q[0] * m_q[0] + m_q[1] * m_q[1] + m_q[2] * m_q[2] + m_q[3] * m_q[3]);
}

/*****************************************************************************
 Prototype    : CQuat.Normalize
 Description  : Normalize quaternion
 Input        : None
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
void CQuat::Normalize()
{
    const double dNorm = Norm();
    operator*=(*this, 1 / dNorm);
}
/**
*  @brief Rotate a vector using this quaternion
*  @param v
*/
/*****************************************************************************
 Prototype    : CQuat.transform
 Description  : Rotate a vector using this quaternion
 Input        : const CQuat &v: Vector stored in the three complex terms
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat CQuat::transform(const CQuat &v)
{
    CQuat &q = *this;
    return q * v * q.Conjugate();
}

/*****************************************************************************
 Prototype    : CQuat.Inverse
 Description  : quaternion inversion
 Input        : None
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat CQuat::Inverse()
{
    //double dNorm2 = m_q[0] * m_q[0] + m_q[1] * m_q[1] + m_q[2] * m_q[2] + m_q[3] * m_q[3];
    double dNorm2 = a() * a() + b() * b() + c() * c() + d() * d();
    double norm_inv = 1.0 / dNorm2;
    return CQuat(norm_inv*a(), -norm_inv*b(), -norm_inv*c(), -norm_inv*d());
}

/*****************************************************************************
 Prototype    : CQuat.quat2Rotation
 Description  : Convert a rotation quaternion to its matrix form
 Input        : None
 Output       : None
 Return Value : 3x3 Rotation matrix
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
 note: 1, The result is not correct if this quaternion is not a member of S(4);
       2, also see: quat2rotm.m in Matlab.
*****************************************************************************/
double* CQuat::quat2Rotation()
{
    double* R = new double[3*3];
    double aa = a() * a();
    double bb = b() * b();
    double cc = c() * c();
    double dd = d() * d();

    R[0] = aa + bb - cc - dd;
    R[3] = 2 * b() * c() + 2 * a() * d();
    R[6] = 2 * b() * d() - 2 * a() * c();

    R[1] = 2 * b() * c() - 2 * a() * d();
    R[4] = aa - bb + cc - dd;
    R[7] = 2 * c() * d() + 2 * a() * b();

    R[2] = 2 * b() * d() + 2 * a() * c();
    R[5] = 2 * c() * d() - 2 * a() * b();
    R[8] = aa - bb - cc + dd;
    return R;
}

/*****************************************************************************
 Prototype    : CQuat.rotation2Quat
 Description  : Create quaternion from matrix
 Input        : double* m: Rotation matrix, should be a member of SO(3).
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
 note: 1, All singularities are handled, provided m belongs to SO(3).
       2, see: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
       3, see rotm2quat.m in Matlab.
*****************************************************************************/
CQuat CQuat::rotation2Quat(double* m)
{

    CQuat Q;
    const double tr = m[0] + m[4] + m[8]; //  trace
    double s = 0;

    if (tr > 0)
    {
      s = 2 * std::sqrt(1.0 + tr);// S=4*qw

      Q.a() = s * 0.25;
      Q.b() = (m[2*3+ 1] - m[1*3+ 2]) / s;
      Q.c() = (m[0*3+ 2] - m[2*3+ 0]) / s;
      Q.d() = (m[1*3+ 0] - m[0*3+ 1]) / s;
    } else if (m[0*3+ 0] > m[1*3+ 1] && m[0*3+ 0] > m[2*3+ 2])
    {
      s = 2 * std::sqrt(1.0 + m[0*3+ 0] - m[1*3+ 1] - m[2*3+ 2]);// S=4*qx

      Q.a() = (m[2*3+ 1] - m[1*3+ 2]) / s;
      Q.b() = s * 0.25;
      Q.c() = (m[0*3+ 1] + m[1*3+ 0]) / s;
      Q.d() = (m[0*3+ 2] + m[2*3+ 0]) / s;
    } else if (m[1*3+ 1] > m[2*3+ 2])
    {
      s = 2 * std::sqrt(1.0 + m[1*3+ 1] - m[0*3+ 0] - m[2*3+ 2]);// S=4*qy

      Q.a() = (m[0*3+ 2] - m[2*3+ 0]) / s;
      Q.b() = (m[0*3+ 1] + m[1*3+ 0]) / s;
      Q.d() = (m[1*3+ 2] + m[2*3+ 1]) / s;
      Q.c() = s * 0.25;
    } else
    {
      s = 2 * std::sqrt(1.0 + m[2*3+ 2] - m[0*3+ 0] - m[1*3+ 1]);// S=4*qz

      Q.a() = (m[1*3+ 0] - m[0*3+ 1]) / s;
      Q.b() = (m[0*3+ 2] + m[2*3+ 0]) / s;
      Q.c() = (m[1*3+ 2] + m[2*3+ 1]) / s;
      Q.d() = s * 0.25;
    }

    return Q;
}

void CQuat::rotation2Quat(const double* m, double* q)
{
    const double tr = m[0] + m[4] + m[8];                   //  trace
    double s = 0;
    if (tr > 0)
    {
      s = 2 * sqrt(1.0 + tr);                               // S=4*qw
      q[0] = s * 0.25;
      q[1] = (m[7] - m[5]) / s;
      q[2] = (m[2] - m[6]) / s;
      q[3] = (m[3] - m[1]) / s;
    } else if (m[0] > m[4] && m[0] > m[8])
    {
      s = 2 * sqrt(1.0 + m[0] - m[4] - m[8]);               // S=4*qx
      q[0] = (m[7] - m[5]) / s;
      q[1] = s * 0.25;
      q[2] = (m[1] + m[3]) / s;
      q[3] = (m[2] + m[6]) / s;
    } else if (m[4] > m[8])
    {
      s = 2 * sqrt(1.0 + m[4] - m[0] - m[8]);               // S=4*qy
      q[0] = (m[2] - m[6]) / s;
      q[1] = (m[1] + m[3]) / s;
      q[2] = s * 0.25;
      q[3] = (m[5] + m[7]) / s;
    } else
    {
      s = 2 * sqrt(1.0 + m[8] - m[0] - m[4]);               // S=4*qz
      q[0] = (m[3] - m[1]) / s;
      q[1] = (m[2] + m[6]) / s;
      q[2] = (m[5] + m[7]) / s;
      q[3] = s * 0.25;
    }
}

/*****************************************************************************
 Prototype    : CQuat.Rotation
 Description  : Create a rotation quaterion
 Input        : double theta: Angle of rotation, radians
                double x    : component of rotation vector
                double y    : Y component
                double z    : Z component
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
CQuat CQuat::Rotation(double theta, double x, double y, double z)
{
    const double dSine   = std::sin(theta / 2);
    const double dCosine = std::cos(theta / 2);

    return CQuat(dCosine, dSine * x, dSine * y, dSine * z);
}

/*****************************************************************************
 Prototype    : CQuat.angle2quat
 Description  : Conversion Euler ('ZYX') to Quaternion
 Input        : double yaw
                double pitch
                double roll
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
 note: see angle2quat.m in Matlab.
*****************************************************************************/
CQuat CQuat::angle2quat(double yaw, double pitch, double roll)
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
    return CQuat(
        cosy * cosp * cosr + siny * sinp * sinr,
        cosy * cosp * sinr - siny * sinp * cosr,
        cosy * sinp * cosr + siny * cosp * sinr,
        siny * cosp * cosr - cosy * sinp * sinr
    );
}

/*****************************************************************************
 Prototype    : CQuat.quat2angle
 Description  : Convert quaternion to rotation angles
 Input        : None
 Output       : None
 Return Value : rpy
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
 note: see quat2angle.m in Matlab.
*****************************************************************************/
double* CQuat::quat2angle()
{
    double* arrAngle = new double[3];
    Normalize();
    double w = a();
    double x = b();
    double y = c();
    double z = d();
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

    return arrAngle;
}

/*****************************************************************************
 Prototype    : CQuat.Print
 Description  : output the value of quaternion
 Input        : None
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
void CQuat::Print()
{
    cout << m_q[0] << "\t";
    cout << m_q[1] << "\t";
    cout << m_q[2] << "\t";
    cout << m_q[3] << endl;
}


