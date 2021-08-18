#ifndef Frame_h_
#define Frame_h_

#include <sstream>
#include "Vector.h"
#include "Rotation.h"
#include <iostream>

class Frame {
public:
    Rotation mOrientation;
    Vector mOrigine;

public:
    inline Frame()
    {
        mOrientation = Rotation::identity() ;
        mOrigine     = Vector(0.0, 0.0, 0.0) ;
    }
    inline Frame(const Rotation &r, const Vector &v)
    {
        mOrientation = r;
        mOrigine = v;
    }

    explicit inline Frame(const Vector &v)
    {
        mOrientation = Rotation::identity();
        mOrigine = v;
    }

    explicit inline Frame(const Rotation &r)
    {
        mOrientation = r;
        mOrigine = Vector::zero();
    }

    explicit inline Frame(double trans[][4])
    {
        mOrientation = Rotation(trans[0][0], trans[0][1], trans[0][2],
                                trans[1][0], trans[1][1], trans[1][2],
                                trans[2][0], trans[2][1], trans[2][2]);
        mOrigine = Vector(trans[0][3], trans[1][3], trans[2][3]);
    }
    explicit inline Frame(const double* trans)
    {
        mOrientation = Rotation(trans[0], trans[1], trans[2],
                                trans[4], trans[5], trans[6],
                                trans[8], trans[9], trans[10]);
        mOrigine = Vector(trans[3], trans[7], trans[11]) ;
    }
    explicit inline Frame(const float* trans)
    {
        //std::cout << "func, line = " << __func__ << "  " << __LINE__ << std::endl;
        mOrientation = Rotation(trans[0], trans[1], trans[2],
                                trans[4], trans[5], trans[6],
                                trans[8], trans[9], trans[10]);
        mOrigine = Vector(trans[3], trans[7], trans[11]) ;
    }

    inline Frame(const Frame &arg)
    {
        mOrientation = arg.mOrientation;
        mOrigine = arg.mOrigine;
    }

    inline static Frame identity()
    {
        return Frame(Rotation::identity(), Vector::zero());
    }

    inline friend Frame operator *(const Frame& lhs,const Frame& rhs)
    {
        return Frame(lhs.mOrientation*rhs.mOrientation, lhs.mOrientation*rhs.mOrigine + lhs.mOrigine);
    }

    // equal with a small epsilon
    inline bool equal(const Frame &other, double eps)
    {
        return (mOrientation.equal(other.mOrientation, eps)
             && mOrigine.equal(other.mOrigine, eps));
    }

    static Frame transX(double x);
    static Frame transY(double y);
    static Frame transZ(double z);

    // constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg convention
    // as in page 4 of handout3_Kinematics-2.pdf
    // alpha, theta: in raddians
    static Frame fromPrepositionDH(double a, double alpha, double d, double theta);
    static Frame fromPostpositionDH(double a, double alpha, double d, double theta);

    // return the homogeneous transformation matrix
    void make4x4(double *d) ;
    void toHomo(double *pHomo) const;
    void fromHomo(const double* pHomo);
    void toHomo(float *pHomo) const;
    void fromHomo(const float* pHomo);

    // get inverse frame
    inline Frame inverse() const
    {
        return Frame(mOrientation.getInverse(), -mOrientation.getInverse(mOrigine));
    }
    // return the vector in *this
    inline Vector operator * (const Vector& v) const
    {
        return mOrientation * v + mOrigine;
    }

    inline Frame& operator=(const Frame& frmOther)
    {
        if (this == &frmOther)
            return *this ;

        mOrientation = frmOther.mOrientation ;
        mOrigine     = frmOther.mOrigine ;

        return *this ;
    }

    inline std::string toString()
    {
        std::stringstream s;
        s<<mOrientation.toString()<<" || "<<mOrigine.toString();
        return s.str();
    }
};
inline void Frame::toHomo(double *pHomo) const
{
    pHomo[0]  = mOrientation.mData[0] ;
    pHomo[1]  = mOrientation.mData[1] ;
    pHomo[2]  = mOrientation.mData[2] ;
    pHomo[3]  = mOrigine.mData[0] ;
    pHomo[4]  = mOrientation.mData[3] ;
    pHomo[5]  = mOrientation.mData[4] ;
    pHomo[6]  = mOrientation.mData[5] ;
    pHomo[7]  = mOrigine.mData[1] ;
    pHomo[8]  = mOrientation.mData[6] ;
    pHomo[9]  = mOrientation.mData[7] ;
    pHomo[10] = mOrientation.mData[8] ;
    pHomo[11] = mOrigine.mData[2] ;
    pHomo[12] = 0.0 ;
    pHomo[13] = 0.0 ;
    pHomo[14] = 0.0 ;
    pHomo[15] = 1.0 ;
}
inline void Frame::toHomo(float *pHomo) const
{
    pHomo[0]  = (float)mOrientation.mData[0] ;
    pHomo[1] = (float)mOrientation.mData[1];
    pHomo[2] = (float)mOrientation.mData[2];
    pHomo[3] = (float)mOrigine.mData[0];
    pHomo[4] = (float)mOrientation.mData[3];
    pHomo[5] = (float)mOrientation.mData[4];
    pHomo[6] = (float)mOrientation.mData[5];
    pHomo[7] = (float)mOrigine.mData[1];
    pHomo[8] = (float)mOrientation.mData[6];
    pHomo[9] = (float)mOrientation.mData[7];
    pHomo[10] = (float)mOrientation.mData[8];
    pHomo[11] = (float)mOrigine.mData[2];
    pHomo[12] = 0.0 ;
    pHomo[13] = 0.0 ;
    pHomo[14] = 0.0 ;
    pHomo[15] = 1.0 ;
}


inline void Frame::fromHomo(const double* pHomo)
{
    mOrigine.mData[0]     = pHomo[3] ;
    mOrigine.mData[1]     = pHomo[7] ;
    mOrigine.mData[2]     = pHomo[11] ;
    mOrientation.mData[0] = pHomo[0] ;
    mOrientation.mData[1] = pHomo[1] ;
    mOrientation.mData[2] = pHomo[2] ;
    mOrientation.mData[3] = pHomo[4] ;
    mOrientation.mData[4] = pHomo[5] ;
    mOrientation.mData[5] = pHomo[6] ;
    mOrientation.mData[6] = pHomo[8] ;
    mOrientation.mData[7] = pHomo[9] ;
    mOrientation.mData[8] = pHomo[10] ;
}

inline void Frame::fromHomo(const float* pHomo)
{
    mOrigine.mData[0]     = pHomo[3] ;
    mOrigine.mData[1]     = pHomo[7] ;
    mOrigine.mData[2]     = pHomo[11] ;
    mOrientation.mData[0] = pHomo[0] ;
    mOrientation.mData[1] = pHomo[1] ;
    mOrientation.mData[2] = pHomo[2] ;
    mOrientation.mData[3] = pHomo[4] ;
    mOrientation.mData[4] = pHomo[5] ;
    mOrientation.mData[5] = pHomo[6] ;
    mOrientation.mData[6] = pHomo[8] ;
    mOrientation.mData[7] = pHomo[9] ;
    mOrientation.mData[8] = pHomo[10] ;
}

#endif
