#ifndef _GEOMETRYUTILS_H_
#define _GEOMETRYUTILS_H_

#include "Vector.h"
#include "Point.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include "Frame.h"
#define PI (3.141592653589793)
using namespace std;
class GeometryUtils
{
public:
    static Vector  getNormal(const Vector& one,const Vector& two) ;
    static double  getVectorAngle(const Vector& one,const Vector& two);
    static double  norm(const Vector& v);
    static double  norm(const double arrd[3]);
    static double* MatrixTranspose(const double* matrix, const int row, const int col);
    static void    MatrixTranspose(const double* Matrix, const int row, const int col, double* MatrixTrans);
    static Vector  pointToVector(const Point& start,const Point& end);
    static Vector  vectorMove(const double* matrix,const Vector& v);
    static double  det(const double* matrix, const int row ,const int col);
    static double  cofactor( const double* matrix, const int row, const int col, const int tartgetRow, const int targetCol);
    static int     MatrixInv(double *matrix, const int n);//recommanded!
    static double* MatrixInv(const double* matrix, const int row, const int col);// not recommanded!
    static void    MatrixInv(const double* matrix, const int row, const int col, double* pResult);// not recommanded!

    static double* rotateX(const double& angle);
    static double* rotateY(const double& angle);
    static double* rotateZ(const double& angle);
    /* BEGIN: Added by lin.lin, 2017/6/7 */
    static double* rotateX2Homo(const double& angle);
    static double* rotateY2Homo(const double& angle);
    static double* rotateZ2Homo(const double& angle);
    static double* trans2Homo(const double& x, const double& y, const double& z);
    static double* transX2Homo(const double& x);
    static double* transY2Homo(const double& y);
    static double* transZ2Homo(const double& z); 
    /* END:   Added by lin.lin, 2017/6/7   PN: */
    
/* BEGIN: Added by lin.lin, 2017/8/11 */
    static void rotateX(const double angle, double* result);
    static void rotateY(const double angle, double* result);
    static void rotateZ(const double angle, double* result);
    static void rotateX2Homo(const double angle, double* result);
    static void rotateY2Homo(const double angle, double* result);
    static void rotateZ2Homo(const double angle, double* result);
    static void trans2Homo(const double x, const double y, const double z, double* result);
    static void transX2Homo(const double x, double* pResult);
    static void transY2Homo(const double y, double* pResult);
    static void transZ2Homo(const double z, double* pResult);
    static void MatrixMultiply(const double a[], const double b[], const int m, const int n, const int k, double c[]);// recommended.
    static double* MatrixMultiply(const double* matrix1,const int& row1,const int& col1,const double* matrix2,const int& row2,const int& col2);//redundant parameters exists; Internal memory application; Not recommended!(comments by lin.lin)
    
/* END:   Added by lin.lin, 2017/8/11   PN: */
    static double* MatrixMult(const double* matrix,const int& row,const int& col,const double& p);
    static double* MatrixPlus(const double* matrix,const int& row,const int& col,const double& p);
    static double* MatrixPlus(const double* matrix1,const double* matrix2,const int& row,const int& col);
    static double Distance(const Vector& a,const Vector& b);
    static double Distance(const double a[3], const double b[3]);
    static void getRYPFromFrame(const Frame& frame,double& roll,double& yaw,double& pitch);
    static void getRPYFromFrame(const Frame& frame, double& roll, double& pitch, double& yaw);

    static double* getRot(const Vector& old_vector,const Vector& new_vector);
    static Vector vectorCross(const Vector& left,const Vector& right);
    static void   vectorCross(double arrdSrcA[3], double arrdSrcB[3], double arrdRelsult[3]); 
    static double vectorDot(const Vector& left,const Vector& right);
    static double vectorDot(double arrd1[3], double arrd2[3]);
    /* BEGIN: Added by lin.lin, 2017/3/28 */
    static void getFrameFromEuler(double roll, double yaw, double pitch, Vector position, Frame& frame);
    /* END:   Added by lin.lin, 2017/3/28   PN: */
    static double DistancePoint2Line(const double point[3], const double origin[3], const double dir[3]);
};

inline double GeometryUtils::norm(const Vector& v) 
{
    return sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
}
// overload norm; by lin.lin 2017.8.11
inline double GeometryUtils::norm(const double arrd[3]) 
{
    return sqrt(arrd[0]*arrd[0] + arrd[1]*arrd[1]  + arrd[2]*arrd[2]);
}
inline double GeometryUtils::Distance(const Vector& a,const Vector& b)
{
    return sqrt((a.x() - b.x()) * (a.x() - b.x()) + 
                (a.y() - b.y()) * (a.y() - b.y()) +
                (a.z() - b.z()) * (a.z() - b.z()));
}

// overload Distance; by lin.lin 2017.11.14
inline double GeometryUtils::Distance(const double a[3], const double b[3])
{
    return sqrt((a[0] - b[0]) * (a[0] - b[0]) + 
                (a[1] - b[1]) * (a[1] - b[1]) +
                (a[2] - b[2]) * (a[2] - b[2]));
}

inline Vector GeometryUtils::vectorCross(const Vector& left,const Vector& right)
{
    return Vector(left.y() * right.z() - left.z()*right.y(), left.z() * right.x() - left.x() * right.z(),
                  left.x() * right.y() - left.y() * right.x());
}
// overload vectorCross; by lin.lin 2017.8.14
inline void GeometryUtils::vectorCross(double arrdSrcA[3], double arrdSrcB[3], double arrdRelsult[3])
{
    arrdRelsult[0] = arrdSrcA[1] * arrdSrcB[2] - arrdSrcA[2] * arrdSrcB[1];
    arrdRelsult[1] = arrdSrcA[2] * arrdSrcB[0] - arrdSrcA[0] * arrdSrcB[2];
    arrdRelsult[2] = arrdSrcA[0] * arrdSrcB[1] - arrdSrcA[1] * arrdSrcB[0];
}
inline double GeometryUtils::vectorDot(const Vector& left,const Vector& right)
{
    return left.mData[0] * right.mData[0] + left.mData[1] * right.mData[1] + left.mData[2] * right.mData[2];
}
// overload vectorDot; by lin.lin 2017.8.14
inline double GeometryUtils::vectorDot(double arrd1[3], double arrd2[3])
{
    return (arrd1[0]*arrd2[0] + arrd1[1]*arrd2[1]  + arrd1[2]*arrd2[2]);
}

#endif
