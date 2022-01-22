#include <iostream>
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;

int main()
{
/** ××××××××××××××××××  （１）block操作
    MatrixXf m(4,4);
    m<<1,2,3,4,
       5,6,7,8,
       9,10,11,12,
       13,14,15,16;

    cout<<"block in middle"<<end;
    cout<<m.block<2,2>(1,1)<<endl<<endl;

    for(int i=1;i<=3;i++)
    {
        cout<<"block of size"<<i<<"x"<<i<<endl;
        cout<<m.block(0,0,i,i)<<endl<<endl;
    }
**/

/** ××××××××××××××××××  （2）block操作当作输入
    Array22f m;
    m<<1,2,
       3,4;

    Array44f a=Array44f::Constant(0.6);
    cout<<"a矩阵"<<endl<<a<<endl<<endl;

    a.block<2,2>(1,1)=m;
    cout<<"m矩阵插入a中心"<<endl<<a<<endl<<endl;

    a.block(0,0,2,3)=a.block(2,1,2,3);
    cout<<endl<<a<<endl<<endl;
**/

/** ××××××××××××××××××  （3）行子式和列的运算
    MatrixXf m(3,3);
    m<<1,2,3,
       4,5,6,
       7,8,9;

    cout<<"第２row"<<m.row(1)<<endl<<endl;

    m.col(2)+=3*m.col(0);
    cout<<"操作后:  "<<m<<endl<<endl;
**/


/** ××××××××××××××××××  （4）边角子矩阵

    Matrix4f m;

    m<<1,2,3,4,
       5,6,7,8,
       9,10,11,12,
       13,14,15,16;

    cout<<"前两列"<<m.leftCols(2)<<endl<<endl;

    cout<<"最下两行"<<m.bottomRows(2)<<endl<<endl;

    m.topLeftCorner(1,3)=m.bottomRightCorner(3,1).transpose();
    cout<<"调整后：　"<<m<<endl<<endl;
**/

/** ××××××××××××××××××  （５）向量子向量操作
    ArrayXf v(6);
    v<<1,2,3,4,5,6;

    cout<<"前三个元素:"<<v.head(3)<<endl<<endl;
    cout<<"后三个元素："<<v.tail(3)<<endl<<endl;
    cout<<"第１个后四个："<<v.segment(1,4)<<endl<<endl;
    v.segment(1,4)*=2;
    cout<<v<<endl;


    


}
