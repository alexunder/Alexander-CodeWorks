#include "Vector.h"

#include <cmath>

#include "MathUtil.h"

double Vector::norm() const
{
    double tmp1;
    double tmp2;
    tmp1 = fabs(mData[0]);
    tmp2 = fabs(mData[1]);
    if (tmp1 >= tmp2) {
        tmp2=fabs(mData[2]);
        if (tmp1 >= tmp2) {
            if (tmp1 == 0) {
                // only to everything exactly zero case, all other are handled correctly
                return 0;
            }
            return tmp1*sqrt(1+sqr(mData[1]/mData[0])+sqr(mData[2]/mData[0]));
        } else {
            return tmp2*sqrt(1+sqr(mData[0]/mData[2])+sqr(mData[1]/mData[2]));
        }
    } else {
        tmp1=fabs(mData[2]);
        if (tmp2 > tmp1) {
            return tmp2*sqrt(1+sqr(mData[0]/mData[1])+sqr(mData[2]/mData[1]));
        } else {
            return tmp1*sqrt(1+sqr(mData[0]/mData[2])+sqr(mData[1]/mData[2]));
        }
    }
}

bool operator ==(const Vector& lhs,const Vector& rhs)
{
    return lhs.mData[0] == rhs.mData[0]
        && lhs.mData[1] == rhs.mData[1]
        && lhs.mData[2] == rhs.mData[2];
}

