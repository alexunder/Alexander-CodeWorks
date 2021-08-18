#include "Frame.h"

/**
 *    r11   r12    r13    p1
 *    r21   r22    r23    p2
 *    r31   r32    r33    p3
 *     0     0      0     1
 **/
void Frame::make4x4(double * d)
{
    int i;
    int j;
    for (i=0;i<3;i++) {
        for (j=0;j<3;j++)
            d[i*4+j]=mOrientation(i,j);
        d[i*4+3] = mOrigine(i);
    }
    for (j=0;j<3;j++)
        d[12+j] = 0.;
    d[15] = 1;
}

Frame Frame::transX(double x)
{
    return Frame(Rotation::identity(),
                 Vector(x, 0, 0));
}

Frame Frame::transY(double y)
{
    return Frame(Rotation::identity(),
                 Vector(0, y, 0));
}

Frame Frame::transZ(double z)
{
    return Frame(Rotation::identity(),
                 Vector(0, 0, z));
}

Frame Frame::fromPrepositionDH(double a, double alpha, double d, double theta)
{
    double ct, st, ca, sa;
    ct = cos(theta);
    st = sin(theta);
    sa = sin(alpha);
    ca = cos(alpha);
    return Frame(Rotation(
                          ct,       -st,     0,
                          st*ca,  ct*ca,   -sa,
                          st*sa,  ct*sa,    ca   ),
                 Vector(a,      -sa*d,  ca*d   )
           );
}

Frame Frame::fromPostpositionDH(double a, double alpha, double d, double theta)
{
    double ct, st, ca, sa;
    ct = cos(theta);
    st = sin(theta);
    sa = sin(alpha);
    ca = cos(alpha);
    return Frame(Rotation(
                          ct,  -st*ca,    st*sa,
                          st,   ct*ca,   -ct*sa,
                          0,       sa,    ca   ),
                 Vector(a*ct,    a*st,    d   )
           );
}


