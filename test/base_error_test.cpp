#include "../include/error/base_error.h"

#include <iostream>

using namespace std;


class ABError : public BaseErrorTerm
{
public:
	ABError(int nErrorDimension, int nX, int nY) 
		:BaseErrorTerm(nErrorDimension)
	{
		
		this->m_nX = nX;
		this->m_nY = nY;
	}


	//The error may be multi-value.
	//This is a pure virtual function.
	virtual Eigen::VectorXd ComputeError()
	{
		OptimizedVariable * pVariableA = this->m_gVariables[0];
		OptimizedVariable * pVariableB = this->m_gVariables[1];

		Eigen::VectorXd mVecA = pVariableA->GetVariable();
		Eigen::VectorXd mVecB = pVariableB->GetVariable();

		Eigen::VectorXd mVecC(1);
		mVecC << 1.0;

		return  (mVecA * this->m_nX) + (mVecB * this->m_nY) + mVecC;
	}

	//The jacobian type should be checked.
	virtual bool ComputeJacobian()
	{

		this->m_gJacobians.clear();
		Eigen::MatrixXd mJacobianA(1, 1);
		Eigen::MatrixXd mJacobianB(1, 1);
		mJacobianA << this->m_nX;
		mJacobianB << this->m_nY;
		this->m_gJacobians.reserve(2);
		this->m_gJacobians.push_back(mJacobianA);
		this->m_gJacobians.push_back(mJacobianB);
		return true;
	}
	
private:


	double m_nX, m_nY;
	
};




int main(){
	
	ABError iTestError(1, 1 , 2);


	Eigen::VectorXd mVariableA(1);
	mVariableA << 1;
	Eigen::VectorXd mVariableB(1);
	mVariableB << 2;

	OptimizedVariable * pVariableA = new OptimizedVariable(mVariableA);
	OptimizedVariable * pVariableB = new OptimizedVariable(mVariableB);

	iTestError.AddVariable(pVariableA);
	iTestError.AddVariable(pVariableB);
	iTestError.ComputeJacobian();
	cout << "iTestError jacobian" << endl;
	vector<Eigen::MatrixXd> gJacobians;
	gJacobians = iTestError.GetJacobians();
	cout << gJacobians[0] << endl << gJacobians[1] << endl;

	return 0;
}