#ifndef _QUATERNIOND_H_
#define _QUATERNIOND_H_
#include "Vector.h"
class Quaterniond
{
public:
    double x;
    double y;
    double z;
    double w;

    inline Quaterniond(const double& x,const double& y,const double& z,const double& w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }

    //TODO
    Quaterniond(const Vector& v,const double& angle);
    inline Quaterniond(const Quaterniond& other)
    {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        this->w = other.w;        
    }
    inline Quaterniond& operator=(const Quaterniond& other)
    {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        this->w = other.w;
        return *this;       
    }
    //TODO
    void toEuler(double& z,double& y,double& x);
    //TODO
    static Quaterniond fromEuler(const double& z,const double& y,const double& x);
    
    //TODO
    inline friend Quaterniond operator*(const Quaterniond& left,const Quaterniond& right);

    //TODO
    Quaterniond conjugate();

    //TODO
    double norm();
        
    //TODO
    Quaterniond revert();

};
#endif
