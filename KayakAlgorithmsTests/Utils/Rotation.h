#ifndef Rotation_h_
#define Rotation_h_

#include <cmath>
#include <sstream>

#include "Vector.h"
#include "MathUtil.h"
class Rotation
{
public:
    double mData[9];

    inline static Rotation identity() { return Rotation(1,0,0,0,1,0,0,0,1); }

    inline Rotation() { *this = Rotation::identity(); }

    /*
     *      xX   yX   zX
     *      xY   yY   zY
     *      xZ   yZ   zZ
     */
    inline Rotation(double xX, double yX, double zX, double xY, double yY, double zY, double xZ, double yZ, double zZ) {
        mData[0] = xX; mData[1]=yX; mData[2]=zX;
        mData[3] = xY; mData[4]=yY; mData[5]=zY;
        mData[6] = xZ; mData[7]=yZ; mData[8]=zZ;
    }

    /*
     *     x    y    z
     */
    inline Rotation(const Vector &x, const Vector &y, const Vector &z) {
        mData[0] = x.mData[0]; mData[3] = x.mData[1]; mData[6] = x.mData[2];
        mData[1] = y.mData[0]; mData[4] = y.mData[1]; mData[7] = y.mData[2];
        mData[2] = z.mData[0]; mData[5] = z.mData[1]; mData[8] = z.mData[2];
    }

    inline Rotation &operator=(const Rotation &arg)
    {
        int count = 9;
        while (count--) mData[count] = arg.mData[count];
        return *this;
    }

    friend Rotation operator *(const Rotation& lhs,const Rotation& rhs);

    //!  Defines a multiplication R*V between a Rotation R and a Vector V.
    // rotate a vector
    inline Vector operator*(const Vector &v) const
    {
        return Vector(
             mData[0]*v.mData[0] + mData[1]*v.mData[1] + mData[2]*v.mData[2],
             mData[3]*v.mData[0] + mData[4]*v.mData[1] + mData[5]*v.mData[2],
             mData[6]*v.mData[0] + mData[7]*v.mData[1] + mData[8]*v.mData[2] 
        );
    }

    inline double operator()(int i, int j) const
    {
        return mData[i*3 + j];
    }

    // Sets the value of *this to its inverse.
    //  1  2  5              1  8  9
    //  8  6  4       to     2  6  7
    //  9  7  3              5  4  3
    inline void setInverse()
    {
        double tmp;
        tmp = mData[1]; mData[1]=mData[3]; mData[3]=tmp;
        tmp = mData[2]; mData[2]=mData[6]; mData[6]=tmp;
        tmp = mData[5]; mData[5]=mData[7]; mData[7]=tmp;
    }

    // equal with a small epsilon
    inline bool equal(const Rotation &other, double eps)
    {
        double diff0 = mData[0] - other.mData[0];
        double diff1 = mData[1] - other.mData[1];
        double diff2 = mData[2] - other.mData[2];
        double diff3 = mData[3] - other.mData[3];
        double diff4 = mData[4] - other.mData[4];
        double diff5 = mData[5] - other.mData[5];
        double diff6 = mData[6] - other.mData[6];
        double diff7 = mData[7] - other.mData[7];
        double diff8 = mData[8] - other.mData[8];
        return ((eps > diff0) && (diff0 > -eps)
            && (eps > diff1) && (diff1 > -eps)
            && (eps > diff2) && (diff2 > -eps)
            && (eps > diff3) && (diff3 > -eps)
            && (eps > diff4) && (diff4 > -eps)
            && (eps > diff5) && (diff5 > -eps)
            && (eps > diff6) && (diff6 > -eps)
            && (eps > diff7) && (diff7 > -eps)
            && (eps > diff8) && (diff8 > -eps)
            );
    }

    // get inverse
    inline Rotation getInverse() const
    {
        Rotation tmp(*this);
        tmp.setInverse();
        return tmp;
    }

    // The same as R.getInverse()*v but more efficient.
    inline Vector getInverse(const Vector& v) const
    {
        return Vector(
             mData[0]*v.mData[0] + mData[3]*v.mData[1] + mData[6]*v.mData[2],
             mData[1]*v.mData[0] + mData[4]*v.mData[1] + mData[7]*v.mData[2],
             mData[2]*v.mData[0] + mData[5]*v.mData[1] + mData[8]*v.mData[2] 
        );
    }

    // generate a rotation matrix for rotation about X
    // 1    0    0
    // 0    cs   -sn
    // 0    sn   cs
    inline static Rotation RotX(double angle)
    {
        double cs=cos(angle);
        double sn=sin(angle);
        return Rotation(1,0,0,0,cs,-sn,0,sn,cs);
    }

    // generate a rotation matrix for rotation about Y
    // cs     0     sn
    // 0      1     0
    // -sn    0     cs
    inline static Rotation RotY(double angle)
    {
        double cs=cos(angle);
        double sn=sin(angle);
        return Rotation(cs,0,sn,0,1,0,-sn,0,cs);
    }

    // generate a rotation matrix for rotation about Z
    // cs     -sn     0
    // sn     cs      0
    // 0      0       1
    inline static Rotation RotZ(double angle)
    {
        double cs=cos(angle);
        double sn=sin(angle);
        return Rotation(cs,-sn,0,sn,cs,0,0,0,1);
    }
    inline std::string toString()
    {
        std::stringstream s;
        s<<"|"<<mData[0]<<" "<<mData[1]<<" "<<mData[2]<<"|";
        s<<"|"<<mData[3]<<" "<<mData[4]<<" "<<mData[5]<<"|";
        s<<"|"<<mData[6]<<" "<<mData[7]<<" "<<mData[8]<<"|";
        return s.str();
    }
    /**
     *
     * Gives back a rotation matrix specified with RPY convention:
     * first rotate around X with roll, then around the
     *              old Y with pitch, then around old Z with yaw
     *
     * Invariants:
     *  - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
     *  - angles + 2*k*PI
     */
    static Rotation fromRPY(double roll,double pitch,double yaw);

    /**  Gives back a vector in RPY coordinates, variables are bound by
     -  -PI <= roll <= PI
     -   -PI <= Yaw  <= PI
     -  -PI/2 <= PITCH <= PI/2

	 convention :
	 - first rotate around X with roll,
	 - then around the old Y with pitch,
	 - then around old Z with yaw

	 if pitch == PI/2 or pitch == -PI/2, multiple solutions for gamma and alpha exist.  The solution where roll==0
	 is chosen.

	 Invariants:
	 - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
	 - angles + 2*k*PI
    **/
    void getRPY(double& roll,double& pitch,double& yaw) const;
};

#endif

