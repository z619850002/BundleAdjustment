#include "../include/error/base_error.h"
#include "../include/optimizer/optimizer.h"
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
	
	ABError iTestError(1, 2 , 1);

	ABError iTestError2(1, 3 , 2);


	ABError iTestError3(1, 4 , 3);


	ABError iTestError4(1, 5 , 4);



	Eigen::VectorXd mVariableA(1);
	mVariableA << 1;
	Eigen::VectorXd mVariableB(1);
	mVariableB << 2;

	OptimizedVariable * pVariableA = new OptimizedVariable(mVariableA);
	pVariableA->SetID(0);
	OptimizedVariable * pVariableB = new OptimizedVariable(mVariableB);
	pVariableB->SetID(1);

	iTestError.AddVariable(pVariableA);
	iTestError.AddVariable(pVariableB);
	iTestError.SetID(2);


	iTestError2.AddVariable(pVariableA);
	iTestError2.AddVariable(pVariableB);
	iTestError2.SetID(3);


	iTestError3.AddVariable(pVariableA);
	iTestError3.AddVariable(pVariableB);
	iTestError3.SetID(4);


	iTestError4.AddVariable(pVariableA);
	iTestError4.AddVariable(pVariableB);
	iTestError4.SetID(5);

	// iTestError.ComputeJacobian();
	// cout << "iTestError jacobian" << endl;
	// vector<Eigen::MatrixXd> gJacobians;
	// gJacobians = iTestError.GetJacobians();
	// cout << gJacobians[0] << endl << gJacobians[1] << endl;



	Optimizer iOptimizer;
	iOptimizer.AddVertex(pVariableA);
	iOptimizer.AddVertex(pVariableB);
	iOptimizer.AddEdge(&iTestError);
	iOptimizer.AddEdge(&iTestError2);
	iOptimizer.AddEdge(&iTestError3);
	iOptimizer.AddEdge(&iTestError4);

	iOptimizer.InitializeOptimziation();


	Block2D * pBlockErrorToA = iOptimizer.GetBlock(&iTestError, pVariableA);
	Block2D * pBlockErrorToB = iOptimizer.GetBlock(&iTestError, pVariableB);

	cout << "BlockA is " << endl 
		 << pBlockErrorToA->m_nStartRow << "   " 
		 << pBlockErrorToA->m_nStartCol << "   " 
		 << pBlockErrorToA->m_nRows << "   " 
		 << pBlockErrorToA->m_nCols << "   "  << endl;



	cout << "BlockB is " << endl 
		 << pBlockErrorToB->m_nStartRow << "   " 
		 << pBlockErrorToB->m_nStartCol << "   " 
		 << pBlockErrorToB->m_nRows << "   " 
		 << pBlockErrorToB->m_nCols << "   "  << endl;


	// iOptimizer.GenerateJacobianMatrix();
	// iOptimizer.GenerateErrorVector();

	iOptimizer.Optimize(100);


	cout << "After optimization: " << endl;
	cout << "Variable A is: " << (*pVariableA) << endl;
	cout << "Variable B is: " << (*pVariableB) << endl;

	return 0;
}