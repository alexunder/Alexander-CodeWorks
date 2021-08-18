#ifndef Vector_h_
#define Vector_h_
#include <sstream>
class Vector {
public:
    double mData[3];

    //! Does not initialise the Vector to zero. use Vector::Zero() or SetToZero for that
    inline Vector() {mData[0]=mData[1]=mData[2] = 0.0;}

    //! Constructs a vector out of the three values x, y and z
    inline Vector(double x, double y, double z)
    {
        mData[0] = x;
        mData[1] = y;
        mData[2] = z;
    }
    
    inline Vector(const double* v)
    {
        mData[0] = v[0];
        mData[1] = v[1];
        mData[2] = v[2];
    }
    inline Vector(const float* v)
    {
        mData[0] = v[0];
        mData[1] = v[1];
        mData[2] = v[2];
    }
    //! Assignment operator. The normal copy by value semantics.
    inline Vector(const Vector& arg)
    {
        mData[0] = arg.mData[0];
        mData[1] = arg.mData[1];
        mData[2] = arg.mData[2];
    }

    //! Assignment operator. The normal copy by value semantics.
    inline Vector& operator = ( const Vector& arg)
    {
        mData[0] = arg.mData[0];
        mData[1] = arg.mData[1];
        mData[2] = arg.mData[2];
        return *this;
    }
    inline Vector& operator = ( const double dArg)
    {
        mData[0] = dArg;
        mData[1] = dArg;
        mData[2] = dArg;
        return *this;
    }
    inline Vector& operator = ( const float fArg)
    {
        mData[0] = fArg;
        mData[1] = fArg;
        mData[2] = fArg;
        return *this;
    }
    inline Vector& operator = ( const double* darrArg)
    {
        mData[0] = darrArg[0];
        mData[1] = darrArg[1];
        mData[2] = darrArg[2];
        return *this;
    }
    inline Vector& operator = ( const float* farrArg)
    {
        mData[0] = farrArg[0];
        mData[1] = farrArg[1];
        mData[2] = farrArg[2];
        return *this;
    }
    // return a zero vector
    inline static Vector zero()
    {
        return Vector(0, 0, 0);
    }

    inline double x() const { return mData[0]; }
    inline double y() const { return mData[1]; }
    inline double z() const { return mData[2]; }
    inline void x(double _x) { mData[0] = _x; }
    inline void y(double _y) { mData[1] = _y; }
    inline void z(double _z) { mData[2] = _z; }

    inline Vector &operator-=(const Vector& arg)
    {
        mData[0] -= arg.mData[0];
        mData[1] -= arg.mData[1];
        mData[2] -= arg.mData[2];
        return *this;
    }

    inline friend Vector operator-(const Vector &lhs, const Vector &rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0]-rhs.mData[0];
        tmp.mData[1] = lhs.mData[1]-rhs.mData[1];
        tmp.mData[2] = lhs.mData[2]-rhs.mData[2];
        return tmp;
    }

    inline friend Vector operator-(const Vector &arg)
    {
        Vector tmp;
        tmp.mData[0] = -arg.mData[0];
        tmp.mData[1] = -arg.mData[1];
        tmp.mData[2] = -arg.mData[2];
        return tmp;
    }

    inline Vector &operator+=(const Vector& arg)
    {
        mData[0] += arg.mData[0];
        mData[1] += arg.mData[1];
        mData[2] += arg.mData[2];
        return *this;
    }

    inline friend Vector operator+(const Vector &lhs, const Vector &rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0]+rhs.mData[0];
        tmp.mData[1] = lhs.mData[1]+rhs.mData[1];
        tmp.mData[2] = lhs.mData[2]+rhs.mData[2];
        return tmp;
    }
    inline friend Vector operator*(const Vector &lhs,const double& rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0] * rhs;
        tmp.mData[1] = lhs.mData[1] * rhs;
        tmp.mData[2] = lhs.mData[2] * rhs;
        return tmp;
    }
    inline friend Vector operator*(const Vector &lhs,const float& rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0] * rhs;
        tmp.mData[1] = lhs.mData[1] * rhs;
        tmp.mData[2] = lhs.mData[2] * rhs;
        return tmp;
    }
    inline friend Vector operator/(const Vector &lhs,const double& rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0] / rhs;
        tmp.mData[1] = lhs.mData[1] / rhs;
        tmp.mData[2] = lhs.mData[2] / rhs;
        return tmp;
    }
    inline friend Vector operator/(const Vector &lhs,const float& rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0] / rhs;
        tmp.mData[1] = lhs.mData[1] / rhs;
        tmp.mData[2] = lhs.mData[2] / rhs;
        return tmp;
    }
    inline friend Vector operator/(const Vector &lhs,const int& rhs)
    {
        Vector tmp;
        tmp.mData[0] = lhs.mData[0] / rhs;
        tmp.mData[1] = lhs.mData[1] / rhs;
        tmp.mData[2] = lhs.mData[2] / rhs;
        return tmp;
    }
    inline double& operator() (int index)
    {
        return mData[index];
    }

    inline void reverseSign()
    {
        mData[0] = -mData[0];
        mData[1] = -mData[1];
        mData[2] = -mData[2];
    }

    // equal with a small epsilon
    inline bool equal(const Vector &other, double eps)
    {
        double xdiff = mData[0] - other.mData[0];
        double ydiff = mData[1] - other.mData[1];
        double zdiff = mData[2] - other.mData[2];
        return ((eps > xdiff) && (xdiff > -eps)
            && (eps > ydiff) && (ydiff > -eps)
            && (eps > zdiff) && (zdiff > -eps));
    }

    // identical
    inline bool operator==(const Vector &other)
    {
        return mData[0] == other.mData[0]
            && mData[1] == other.mData[1]
            && mData[2] == other.mData[2];
    }
    inline std::string toString() 
    {
        std::stringstream s;
        s<<"|"<<mData[0]<<"|";
        s<<"|"<<mData[1]<<"|";
        s<<"|"<<mData[2]<<"|";
        return s.str(); 
    }
    inline bool operator!=(const Vector &other) { return !(*this == other); }

    friend bool operator ==(const Vector& lhs,const Vector& rhs);

    double norm() const;
};

#endif

