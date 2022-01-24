#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
using namespace std;
using namespace Eigen;

MatrixXd ReadData(istream & data, int a, int b)
{
	MatrixXd m_matrix(a, b);
	VectorXd hang(a);
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
    /**
    ifstream in("data.txt",ios::in);
    if(!in)
    {
        return 0;
    }
    
    MatrixXd m_matrix=ReadData(in,7,2);
    cout<<m_matrix<<endl;
    system("pause");
    
    return 0;
    **/

	ifstream in("data.txt", ios::in);
	if (!in)
	{
		return 0;
	}
	MatrixXd m_matrix = ReadData(in, 7, 2);
	cout << m_matrix << endl;



    /**
   MatrixXf A = MatrixXf::Random(3, 2);
   cout << "Here is the matrix A:\n" << A << endl;
   VectorXf b = VectorXf::Random(3);
   cout << "Here is the right hand side b:\n" << b << endl;
   
   // SVD decomposition
   cout << "The least-squares solution is:\n"
        << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;

  // QR decomposition
  cout << "The solution using the QR decomposition is:\n"
     << A.colPivHouseholderQr().solve(b) << endl;

  // normal equations
  cout << "The solution using normal equations is:\n"
     << (A.transpose() * A).ldlt().solve(A.transpose() * b) << endl;
   **/


}
