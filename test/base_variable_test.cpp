#include <iostream>
#include "../include/variable/base_variables.h"
using namespace std;


int main(){
	cout << "Hello, world!" << endl;
	OptimizedVariable iVariable(3);
	Eigen::VectorXd mVec(3);
	mVec << 1.0 , 2.0 , 3.0;
	iVariable.SetVariable(mVec);
	cout << "iVariable is: " << endl << iVariable << endl;

	Eigen::Vector4d mVec2(1 , 2 , 2 ,  3);
	OptimizedVariable iVariable2(mVec2);
	cout << "iVariable2 is: " << endl << iVariable2 << endl;


	cv::Mat mInitialMat = (cv::Mat_<double>(3 , 2) << 1.0 , 2.0 , 3.0 , 4.0 , 4.0 , 4.0);
	OptimizedVariable iVariable3(mInitialMat);
	cout << "iVariable3 is: " << endl << iVariable3 << endl;


	iVariable3.SetVariable(mVec2);
	iVariable2.SetVariable(mInitialMat);

	cout << "iVariable3 is: " << endl << iVariable3 << endl;
	cout << "iVariable2 is: " << endl << iVariable2 << endl;


	return 0;
}