#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
using namespace std;
using namespace Eigen;

MatrixXf ReadData(istream & data, int a, int b)
{
	MatrixXf m_matrix(a, b);
	VectorXf hang(b);
	for (int j = 0; j < a; j++)//共a 行
	{
		for (int i = 0; i < b; i++)//共b 列 组成一行
		{
			data >> hang(i);
		}
		m_matrix.row(j) = hang;
        
	}
	return m_matrix;
}

int main()
{
    //读取数据到　matrix1
	ifstream in("data.txt", ios::in);
	if (!in)
	{
		return 0;
	}
	MatrixXf matrix1 = ReadData(in, 100, 2);
	//cout << matrix1 << endl;

   //A=(x,1)
   MatrixXf A = MatrixXf::Constant(100,2,1);
   A.col(0)=matrix1.col(0);
   //cout << "Here is the matrix A:\n" << A << endl;
   //b=(y)
   VectorXf b = matrix1.col(1);
   //cout << "Here is the right hand side b:\n" << b << endl;
   
   // SVD decomposition

   Vector2f x =A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
   cout << "The least-squares solution is:\n"<< x << endl;

  // QR decomposition

  //Vector2f x=A.colPivHouseholderQr().solve(b);
  //cout << "The solution using the QR decomposition is:\n"<< A.colPivHouseholderQr().solve(b) << endl;

  // normal equations
  //Vector2f x=(A.transpose() * A).ldlt().solve(A.transpose() * b);
  //cout << "The solution using normal equations is:\n"<< x << endl;


  double relative_error = (A*x - b).norm() / b.norm();
  cout << "The relative error is:\n" << relative_error << endl;
  Eigen::MatrixXf cond = A.bdcSvd(ComputeThinU | ComputeThinV).singularValues();
  cout<<"奇异值为："<<cond<<endl;
  cout<<"data的矩阵条件数为："<<cond(0)/cond(1)<<endl;
  return 0;

}
