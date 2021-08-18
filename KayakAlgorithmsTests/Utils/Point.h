#ifndef _POINT_H_
#define _POINT_H_

#include <cmath>
class Point
{
public:
    double mData[3];
    Point(double x = 0,double y = 0,double z = 0)
    {
        mData[0] = x;
        mData[1] = y;
        mData[2] = z;
    }
    Point(const Point& p)
    {
        mData[0] = p.mData[0];
        mData[1] = p.mData[1];
        mData[2] = p.mData[2];
    }

    inline const double x()const {return mData[0];}
    inline const double y()const {return mData[1];}
    inline const double z()const {return mData[2];}
    
    inline double x(double _x){mData[0] = _x;}
    inline double y(double _y){mData[1] = _y;}
    inline double z(double _z){mData[2] = _z;}
    
    // identical
    inline bool operator==(const Point& other)
    {
        return fabs(mData[0] -  other.mData[0]) < 0.00001
            && fabs(mData[1] -  other.mData[1]) < 0.00001
            && fabs(mData[2] -  other.mData[2]) < 0.00001;
    }
};
#endif
