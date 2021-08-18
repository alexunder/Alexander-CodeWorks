#include "GeometryUtils.h"

Vector GeometryUtils::getNormal(const Vector& one,const Vector& two) 
{
    Vector normal(one.y() * two.z() - one.z()*two.y(), one.z() * two.x() - one.x() * two.z(),
                  one.x() * two.y() - one.y() * two.x());
    double t = sqrt(normal.x()*normal.x() + normal.y()*normal.y() + 
                    normal.z()*normal.z());
normal.x(normal.x() / t);
    normal.y(normal.y() / t);
    normal.z(normal.z() / t);

    return normal;
}
double GeometryUtils::getVectorAngle(const Vector& one,const Vector& two) 
{
    return acos((one.x() * two.x() + one.y() * two.y() + one.z() * two.z()) / \
                (GeometryUtils::norm(one)*GeometryUtils::norm(two)));
}

double* GeometryUtils::MatrixTranspose(const double* matrix, const int row, const int col) 
{
    double* result = new double[row*col];
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            result[j*row+i] = matrix[i*col+j];
        }
    }
    return result;
} 
// overload MatrixTranspose; by lin.lin 2017.8.11
void GeometryUtils::MatrixTranspose(const double* Matrix, const int row, const int col, double* MatrixTrans)
{
    int i, j;
    for(i=0; i<row; ++i)
    {
        for(j=0; j<col; ++j)
        {
            MatrixTrans[j*row+i] = Matrix[i*col+j];
        }
    }
}
//void GeometryUtils::revertMatrix(double* in,int row,int col,double* out) 
//{
//    for(int i=0;i<row;++i)
//    {
//        for(int j=0;j<col;++j)
//        {
//            out[j*row+i] = in[i*col+j];
//        }
//    } 
//} 
Vector GeometryUtils::pointToVector(const Point& start,const Point& end)
{
    return Vector(end.x() - start.x(),end.y() - start.y(),end.z() - start.z());
}

Vector GeometryUtils::vectorMove(const double* matrix,const Vector& v)
{
    Vector result;  
    for(int i=0;i<3;++i)
    {
        double sum = 0;
        for(int j=0;j<3;++j)
        {
            sum += (matrix[i*4+j] * v.mData[j]);
        }
        result.mData[i] = sum;
    }
    result.mData[0] += matrix[3];
    result.mData[1] += matrix[7];
    result.mData[2] += matrix[11];
    return result;
}

double GeometryUtils::det(const double* matrix, const int row, const int col)
{
    double *temp = new double[row*col];
    memcpy(temp,matrix,sizeof(double)*col*row);
    for(int i=0;i<col;++i)
    {
        for(int j=i+1;j<row;++j)
        {
            if(temp[i*col+i] == 0)
                return 0;
            double factor = temp[j*col+i]/temp[i*col+i];
            for(int k=0;k<col;++k)
               temp[j*col+k] = temp[i*col+k]*(0-factor) + temp[j*col+k];
        }
    }
    double result = 1;
    for(int i=0;i<row;++i)
    {
        result *= temp[i*col+i];
    }
    delete[] temp;
    return result; 
}

/************************************************************* 
 功能：     矩阵求逆函数: 用全选主元高斯-约当(Gauss-Jordan)消去法求n阶实矩阵A的逆矩阵                               
 参数列表： a[in, out]: 输入待求逆的矩阵; 输出矩阵的逆
            n[in]:      矩阵的维数
 返回值：   1 - 求逆成功; 0 - 求逆失败，表示A奇异                                                  
 创建者：   林 琳                                                
 创建时间： 2017/09/13                                           
 修改者：                                                        
 修改原因：                                                     
 修改时间:                                                      
 说明:     推荐使用     
**************************************************************/ 
int GeometryUtils::MatrixInv(double *a, const int n)
{ 
    int *is, *js, i, j, k, l, u, v;
    double d, p;
    is = (int *)malloc(n*sizeof(int));
    js = (int *)malloc(n*sizeof(int));
    for (k=0; k<=n-1; k++)
    { 
        d = 0.0;
        for (i=k; i<=n-1; i++)
        {
            for (j=k; j<=n-1; j++)
            { 
                l = i * n + j; 
                p = fabs(a[l]);
                if (p > d) 
                { 
                    d = p; 
                    is[k] = i; 
                    js[k] = j;
                }
            }
        }  
        if (d+1.0 == 1.0)
        { 
            free(is); 
            free(js); 
            printf("err**not inv\n");
            return(0);
        }
        if (is[k] != k)
        {
            for (j=0; j<=n-1; j++)
            { 
                u = k * n + j; 
                v = is[k] * n + j;
                
                p = a[u]; 
                a[u] = a[v]; 
                a[v] = p;
            }
        }
        if (js[k] != k)
        {
            for (i=0; i<=n-1; i++)
            { 
                u = i * n + k; 
                v = i * n + js[k];
                
                p = a[u]; 
                a[u] = a[v]; 
                a[v] = p;
            }
        }
        l = k * n + k;
        a[l] = 1.0 / a[l];
        for (j=0; j<=n-1; j++)
        {
            if (j!=k)
            { 
                u = k * n + j; 
                a[u] = a[u] * a[l];
            }
        }
        
        for (i=0; i<=n-1; i++)
        {
            if (i != k)
            {
                for (j=0; j<=n-1; j++)
                {
                    if (j != k)
                    { 
                        u = i * n + j;
                        a[u] = a[u] - a[i*n+k] * a[k*n+j];
                    }
                }  
            }  
        }
        for (i=0; i<=n-1; i++)
        {
            if (i != k)
            { 
                u = i * n + k; 
                a[u] = -a[u] * a[l];
            }
        }
        
    }
    for (k=n-1; k>=0; k--)
    { 
        if (js[k] != k)
        {
            for (j=0; j<=n-1; j++)
            { 
                u = k * n + j; 
                v = js[k] * n + j;
                
                p = a[u]; 
                a[u] = a[v]; 
                a[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i=0; i<=n-1; i++)
            { 
                u = i * n + k; 
                v = i * n + is[k];
                
                p = a[u]; 
                a[u] = a[v]; 
                a[v] = p;
            }
        }
    }
    free(is); 
    free(js);
    return(1);
}

/*****************************************************************************
 Prototype    : GeometryUtils.MatrixInv
 Description  : inverse of a matrix
 Input        : const double* matrix  : orginal matrix
                const int& row        : total row of the matrix
                const int& col        : total column of the matrix
 Output       : None
 Return Value : double*               : the inverse of the orginal matrix
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : 
    Modification : Created function
  note: this version is not recommanded! pls use int GeometryUtils::MatrixInv(double *a, const int n)
*****************************************************************************/
double* GeometryUtils::MatrixInv(const double* matrix, const int row, const int col)
{
    double det = GeometryUtils::det(matrix,row,col); 
    if(det == 0)
        return nullptr;
    double* result = new double[row*col];
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            result[j*col+i] = GeometryUtils::cofactor(matrix, row, col, i, j) / det;
        }
    }
    return result;
}
/*****************************************************************************
 Prototype    : GeometryUtils.MatrixInv
 Description  : inverse of a matrix 
 Input        : const double* matrix  : orginal matrix  
                const int row         : total row of the matrix  
                const int col         : total column of the matrix  
                double* pResult       : the inverse of the orginal matrix  
 Output       : None  
 Return Value : void
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2017/9/22
    Author       : lin.lin
    Modification : Created function
note: this version is not recommanded! pls use int GeometryUtils::MatrixInv(double *a, const int n)
*****************************************************************************/
void GeometryUtils::MatrixInv(const double* matrix, const int row, const int col, double* pResult)
{
    double det = GeometryUtils::det(matrix,row,col); 
    if(det == 0){
        //std::cout << "MatrixInv: det == 0" << std::endl;
        pResult = nullptr;
    }
    //double* result = new double[row*col];
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            pResult[j*col+i] = GeometryUtils::cofactor(matrix,row,col,i,j)/det;
        }
    }
    //return result;
}
double GeometryUtils::cofactor(const double* matrix,const int row, const int col, const int targetRow, const int targetCol)
{
    double* result = new double[(row-1) * (col-1)];
    int index = 0;
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            if(i == targetRow || j == targetCol)
                continue;
            result[index++] = matrix[i*col+j];
        }
    }
    double det = GeometryUtils::det(result,row-1, col-1);
    delete[] result;
    
    return pow(-1,targetRow+targetCol)*det;
}
/*****************************************************************************
 Prototype    : GeometryUtils.rotateX
 Description  : turn angles around X axis (pitch)
 Input        : angle    : angle (Unit: radian measure)
 Output       : None
 Return Value : rotation matrix 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : 
    Modification : Created function
*****************************************************************************/
double* GeometryUtils::rotateX(const double& angle)
{
    double* result = new double[3*3];
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0; 
    result[4] = cos(angle);
    result[5] = -sin(angle);
    result[6] = 0;
    result[7] = sin(angle);
    result[8] = cos(angle);
    
    return result;
}
// overload rotateX; by lin.lin, 2017/8/11
// note: to avoid memory allocation from heap. Protect memory leap.
void GeometryUtils::rotateX(const double angle, double* result)
{
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0; 
    result[4] = cos(angle);
    result[5] = -sin(angle);
    result[6] = 0;
    result[7] = sin(angle);
    result[8] = cos(angle);
}
/*****************************************************************************
 Prototype    : GeometryUtils.rotateY
 Description  : turn angles around Y axis (yaw)
 Input        : angle    : angle (Unit: radian measure)
 Output       : None
 Return Value : rotation matrix 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : 
    Modification : Created function
*****************************************************************************/
double* GeometryUtils::rotateY(const double& angle)
{
    double* result = new double[3*3];
    result[0] = cos(angle);
    result[1] = 0;
    result[2] = sin(angle);
    result[3] = 0;
    result[4] = 1;
    result[5] = 0;
    result[6] = -sin(angle);
    result[7] = 0;
    result[8] = cos(angle); 

    return result;
}
// overload rotateY; by lin.lin, 2017/8/11
void GeometryUtils::rotateY(const double angle, double* result)
{
    result[0] = cos(angle);
    result[1] = 0;
    result[2] = sin(angle);
    result[3] = 0;
    result[4] = 1;
    result[5] = 0;
    result[6] = -sin(angle);
    result[7] = 0;
    result[8] = cos(angle);
}
/*****************************************************************************
 Prototype    : GeometryUtils.rotateZ
 Description  : turn angles around Z axis (roll)
 Input        : angle    : angle (Unit: radian measure)
 Output       : None
 Return Value : rotation matrix 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : 
    Modification : Created function
*****************************************************************************/
double* GeometryUtils::rotateZ(const double& angle)
{
    double* result = new double[3*3];
    result[0] = cos(angle);
    result[1] = -sin(angle);
    result[2] = 0;
    result[3] = sin(angle);
    result[4] = cos(angle); 
    result[5] = 0;
    result[6] = 0;
    result[7] = 0;
    result[8] = 1;

    return result; 
}
// overload rotateZ; by lin.lin, 2017/8/11
void GeometryUtils::rotateZ(const double angle, double* result)
{
    result[0] = cos(angle);
    result[1] = -sin(angle);
    result[2] = 0;
    result[3] = sin(angle);
    result[4] = cos(angle); 
    result[5] = 0;
    result[6] = 0;
    result[7] = 0;
    result[8] = 1;
}
double* GeometryUtils::rotateX2Homo(const double& angle)
{
    double* result = new double[4*4];
    
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0; 

    result[4] =  0;
    result[5] =  cos(angle);
    result[6] = -sin(angle);
    result[7] =  0;

    result[8] =  0;
    result[9] =  sin(angle);
    result[10] = cos(angle);
    result[11] = 0;

    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 
    
    return result;
}
// overload rotateX2Homo; by lin.lin, 2017/8/11
void GeometryUtils::rotateX2Homo(const double angle, double* result)
{
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0; 

    result[4] =  0;
    result[5] =  cos(angle);
    result[6] = -sin(angle);
    result[7] =  0;

    result[8] =  0;
    result[9] =  sin(angle);
    result[10] = cos(angle);
    result[11] = 0;

    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 
}
double* GeometryUtils::rotateY2Homo(const double& angle)
{
    double* result = new double[4*4];
    
    result[0] = cos(angle);
    result[1] = 0;
    result[2] = sin(angle);
    result[3] = 0;

    result[4] = 0;
    result[5] = 1;
    result[6] = 0;
    result[7] = 0;

    
    result[8]  = -sin(angle);
    result[9]  =  0;
    result[10] =  cos(angle); 
    result[11] =  0; 

    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 
    
    return result;    
}
// overload rotateY2Homo; by lin.lin, 2017/8/11
void GeometryUtils::rotateY2Homo(const double angle, double* result)
{
    result[0] = cos(angle);
    result[1] = 0;
    result[2] = sin(angle);
    result[3] = 0;

    result[4] = 0;
    result[5] = 1;
    result[6] = 0;
    result[7] = 0;

    
    result[8]  = -sin(angle);
    result[9]  =  0;
    result[10] =  cos(angle); 
    result[11] =  0; 

    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 
}
double* GeometryUtils::rotateZ2Homo(const double& angle)
{
    double* result = new double[4*4];
    
    result[0] =  cos(angle);
    result[1] = -sin(angle);
    result[2] =  0;
    result[3] =  0;
    
    result[4] = sin(angle);
    result[5] = cos(angle); 
    result[6] = 0;
    result[7] = 0;

    result[8] =  0;
    result[9] =  0;
    result[10] = 1;
    result[11] = 0;    
    
    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 

    return result; 
}
// overload rotateZ2Homo; by lin.lin, 2017/8/11
void GeometryUtils::rotateZ2Homo(const double angle, double* result)
{
    result[0] =  cos(angle);
    result[1] = -sin(angle);
    result[2] =  0;
    result[3] =  0;
    
    result[4] = sin(angle);
    result[5] = cos(angle); 
    result[6] = 0;
    result[7] = 0;

    result[8] =  0;
    result[9] =  0;
    result[10] = 1;
    result[11] = 0;    
    
    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1;     
}
double* GeometryUtils::trans2Homo(const double& x, const double& y, const double& z)
{
    double* result = new double[4*4];
    
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = x;
    
    result[4] = 0;
    result[5] = 1; 
    result[6] = 0;
    result[7] = y;

    result[8]  = 0;
    result[9]  = 0;
    result[10] = 1;
    result[11] = z;    
    
    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 

    return result; 
}
// overload trans2Homo; by lin.lin, 2017/8/11
void GeometryUtils::trans2Homo(const double x, const double y, const double z, double* result)
{
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = x;
    
    result[4] = 0;
    result[5] = 1; 
    result[6] = 0;
    result[7] = y;

    result[8]  = 0;
    result[9]  = 0;
    result[10] = 1;
    result[11] = z;    
    
    result[12] = 0;
    result[13] = 0;
    result[14] = 0;
    result[15] = 1; 
}
double* GeometryUtils::transX2Homo(const double& x)
{
    double* result = GeometryUtils::trans2Homo(x, 0, 0);
    return result;
}
// overload transX2Homo; by lin.lin, 2017/8/11
void GeometryUtils::transX2Homo(const double x, double* pResult)
{
    GeometryUtils::trans2Homo(x, 0, 0, pResult);
}

double* GeometryUtils::transY2Homo(const double& y)
{
    double* result = GeometryUtils::trans2Homo(0, y, 0);
    return result;
}
// overload transY2Homo; by lin.lin, 2017/8/11
void GeometryUtils::transY2Homo(const double y, double* pResult)
{
    GeometryUtils::trans2Homo(0, y, 0, pResult);
}

double* GeometryUtils::transZ2Homo(const double& z)
{
    double* result = GeometryUtils::trans2Homo(0, 0, z);
    return result;
}
// overload transZ2Homo; by lin.lin, 2017/8/11
void GeometryUtils::transZ2Homo(const double z, double* pResult)
{
    GeometryUtils::trans2Homo(0, 0, z, pResult);
}
/*****************************************************************************
 Prototype    : MatrixMultiply
 Description  : work out the production of matrix A (m*n) and matrix B (n*k)
                C = A * B 
 Input        : a  : matrix A
                b  : matrix B
                m  : the row number of A
                n  : the column number of A, also row number of B
                k  : the column number of B
 Output       : c  : return the production of A * B
 Return Value : void
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2017/4/10
    Author       : lin.lin
    Modification : Created function
  note: Recommended.
*****************************************************************************/
void GeometryUtils::MatrixMultiply(const double a[], const double b[], const int m, const int n, const int k, double c[])
{ 
    int i, j, l, u;
    for (i=0; i<=m-1; ++i)
    {
        for (j=0; j<=k-1; ++j)
        { 
            u = i * k + j;                                  //u = (i, j)
            c[u] = 0.0;
            for (l=0; l<=n-1; ++l)
            {
                c[u] = c[u] + a[i*n+l] * b[l*k+j];          //c(i,j) = c(i,j) + a(i,l) * b(l, j)
            }
        }
    }
    return;
}

/*****************************************************************************
 Prototype    : GeometryUtils.MatrixMultiply
 Description  : multiplication of matrices
 Input        : const double* matrix1 : matrix 1
                const int& row1       : rows of matrix 1       
                const int& col1       : columns of matrix 1
                const double* matrix2 : matrix 2
                const int& row2       : rows of matrix 2     
                const int& col2       : columns of matrix 2    
 Output       : None
 Return Value : double*               : new matrix which equal to matrix 1 * matrix 2
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : deming.Qiao
    Modification : Created function

*****************************************************************************/
double* GeometryUtils::MatrixMultiply(const double* matrix1,const int& row1,const int& col1,const double* matrix2,const int& row2,const int& col2)
{
    double *result= new double[row1*col2];
    memset(result,0,sizeof(double)*row1*col2);
    for(int i=0;i<row1;++i)
    {
        for(int j=0;j<col2;++j)
        {
            for(int k=0;k<col1;++k)
            {
                result[i*col2+j] += matrix1[i*col1+k]*matrix2[k*col2+j];
            }
        }
    }
    return result;
}

/*****************************************************************************
 Prototype    : GeometryUtils.getRYPFromFrame
 Description  : get roll, yaw and pitch from frame.
 Input        : frame :    
 Output       : roll  : roll(z); (radian measure)
                yaw   : yaw(y);  (radian measure)   
                pitch : pitch(x);(radian measure)
 Return Value : void
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : 
    Modification : Created function
 note: 
 1, factor as RyRxRz
 2, also see matrix command: eul2rotm, rotm2eul
*****************************************************************************/
void GeometryUtils::getRYPFromFrame(const Frame& frame,double& roll,double& yaw,double& pitch)
{
    //use formular from 
    //www.geometrictools.com/Documentation/EulerAngles.pdf (2.3 Factor as RyRxRz)
    double angle = frame.mOrientation.mData[5];
    if( angle < 1)
    {
        if( angle > -1)
        {
            pitch = asin(0-angle);
            yaw = atan2(frame.mOrientation.mData[2],frame.mOrientation.mData[8]);
            roll = atan2(frame.mOrientation.mData[3],frame.mOrientation.mData[4]);
        }
        else
        {
            pitch = PI / 2;
            yaw = -atan2(-frame.mOrientation.mData[1],frame.mOrientation.mData[0]);
            roll = 0;
        }
    }
    else
    {
        pitch = -PI / 2;
        yaw = atan2(-frame.mOrientation.mData[1],frame.mOrientation.mData[0]);
        roll = 0;    
    }
}
/*****************************************************************************
 Prototype    : GeometryUtils.getRPYFromFrame
 Description  : get roll, pitch and yaw from frame.
 Input        : frame :    
 Output       : roll  : roll(z); (radian measure); (-PI/2, PI/2)
                pitch : pitch(y);(radian measure); [-PI/2, PI/2]
                yaw   : yaw(x);  (radian measure); (-PI/2, PI/2)   
 Return Value : void
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 
    Author       : 
    Modification : Created function
 note: 
 1, factor as RzRyRx
 2, also see 2.6 Facotor as RzRyRx in "Euler Angle Formulas.pdf"
*****************************************************************************/
void GeometryUtils::getRPYFromFrame(const Frame& frame, double& roll, double& pitch, double& yaw)
{
    double thetaX, thetaY, thetaZ;
    double angle = frame.mOrientation.mData[6];
    if( angle < 1)
    {
        if( angle > -1)
        {
            thetaY = asin(0-angle);
            thetaZ = atan2(frame.mOrientation.mData[3], frame.mOrientation.mData[0]);
            thetaX = atan2(frame.mOrientation.mData[7], frame.mOrientation.mData[8]);
        }
        else
        {
            thetaY = PI / 2;
            thetaZ = -atan2(-frame.mOrientation.mData[5], frame.mOrientation.mData[4]);
            thetaX = 0;
        }
    }
    else
    {
        thetaY = -PI / 2;
        thetaZ = atan2(-frame.mOrientation.mData[5], frame.mOrientation.mData[4]);
        thetaX = 0;    
    }    
    // get roll, pitch, yaw
    roll  = thetaZ;
    pitch = thetaY;
    yaw   = thetaX;
}
/* BEGIN: Added by lin.lin, 2017/3/28 */

/*****************************************************************************
 Prototype    : GeometryUtils.getFrameFromEuler
 Description  : get frame from Euler angles (roll, yaw, pitch)
 Input        : roll  : roll angle (Unit: radian measure)
                yaw   : yaw angle  (Unit: radian measure)
                pitch : pitch angle(Unit: radian measure) 
                position: vector (3*1)                 
 Output       : frame : frame
 Return Value : void
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2017/3/28
    Author       : lin.lin
    Modification : Created function
*****************************************************************************/
void GeometryUtils::getFrameFromEuler(double roll, double yaw, double pitch, Vector position, Frame& frame)
{
    //turn roll to rotation matrix
    double* rz = GeometryUtils::rotateZ(roll);
    //turn yaw to rotation matrix
    double* ry = GeometryUtils::rotateY(yaw);
    //turn pitch to rotation matrix
    double* rx = GeometryUtils::rotateX(pitch);
    //multiply 3 rotation matrice to 1 rotation matrix (Ry*Rx*Rz)
    double* Rxz  = GeometryUtils::MatrixMultiply(rx, 3, 3, rz, 3, 3);
    double* Ryxz = GeometryUtils::MatrixMultiply(ry, 3, 3, Rxz,3, 3);
    // turn rotation matrix to frame
    //memcpy(flangeFrame.mOrientation.mData, matrix, sizeof(double) * 9);
    memcpy(frame.mOrientation.mData, Ryxz, sizeof(double)*9);
    frame.mOrigine = position;
    delete[] rz; 
    delete[] ry; 
    delete[] rx; 
    
    delete[] Rxz;
    delete[] Ryxz;
}

/* END:   Added by lin.lin, 2017/3/28 */

// matrix multiply number
double* GeometryUtils::MatrixMult(const double* matrix,const int& row,const int& col,const double& p)
{
    double* result = new double[row * col];
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            result[i*col+j] = matrix[i*col+j] * p;
        }
    }
    return result;
}
double* GeometryUtils::MatrixPlus(const double* matrix,const int& row,const int& col,const double& p)
{
    double* result = new double[row * col];
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            result[i*col + j] = matrix[i*col+j] + p;
        }
    }
    return result;
}
double* GeometryUtils::MatrixPlus(const double* matrix1,const double* matrix2,const int& row,const int& col)
{
    double* result = new double[3*3];
    for(int i=0;i<row;++i)
    {
        for(int j=0;j<col;++j)
        {
            int index = i * col + j;
            result[index] = matrix1[index] + matrix2[index]; 
        }
    } 
    return result;
}
double* GeometryUtils::getRot(const Vector& old_vector,const Vector& new_vector)
{
    Vector temp_old = old_vector / GeometryUtils::norm(old_vector);
    Vector temp_new = new_vector / GeometryUtils::norm(new_vector);
    Vector k = GeometryUtils::vectorCross(temp_old,temp_new);  
    if(fabs(k.mData[0]) < 0.000001 && 
        fabs(k.mData[1]) < 0.000001 && 
        fabs(k.mData[2]) < 0.000001)
    {
        double* result = new double[3*3];
        memset(result,0,sizeof(double)*3*3);
        result[0] = result[4] = result[8] = 1;
        return result;
    }   
    double costheta = GeometryUtils::vectorDot(temp_old,temp_new);
    double R[]={
        0,-k.mData[2],k.mData[1],
        k.mData[2],0,-k.mData[0],
        -k.mData[1],k.mData[0],0
    };    
    //costheta * eye(3)
    double eye[] = {
        1,0,0,
        0,1,0,
        0,0,1
    }; 
    double* cosMultEye = GeometryUtils::MatrixMult(eye,3,3,costheta);
    
    //k*k'
    double* kMultkRevert = GeometryUtils::MatrixMultiply(k.mData,3,1,k.mData,1,3);
    
    //k*k'*(1-costheta) / sum(k.^2)
    double p = (1-costheta) / (k.mData[0] * k.mData[0] + k.mData[1] * k.mData[1] + k.mData[2] * k.mData[2]);
    double* temp1 = GeometryUtils::MatrixMult(kMultkRevert,3,3,p);

    //costhete*eye(3) + R
    double* temp2 = GeometryUtils::MatrixPlus(cosMultEye,R,3,3);
    
    //costheta*eye(3) + R + k*k'*(1-costheta)/sum(k.^2);
    double* result = GeometryUtils::MatrixPlus(temp2,temp1,3,3);


    delete[] cosMultEye;   
    delete[] kMultkRevert;   
    delete[] temp1;
    delete[] temp2;
    
    return result;
}


/**
* \brief DistancePoint2Line It calculates the minimal distance between the point
*                and the line in theCartesian coordinate.
*
* \param point[3]  The point's coordinate.
* \param origin[3]  The line's origin point.
* \param dir[3]  The line's direction vector
*
*  It is referenced in Mathematics for 3D Game Programming and Computer Graphics Third Edition's
*  5.1.1 section.
*/
double GeometryUtils::DistancePoint2Line(const double point[3], const double origin[3], const double dir[3])
{
    double po[3];
    po[0] = point[0] - origin[0];
    po[1] = point[1] - origin[1];
    po[2] = point[2] - origin[2];

    double long_edge = po[0]*po[0] +  po[1]*po[1] +  po[2]*po[2];
    double short_edge = pow(vectorDot(po, dir), 2) / vectorDot(dir, dir);
    return sqrt(long_edge - short_edge);
}
