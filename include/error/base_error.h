#ifndef BASE_ERROR_H_
#define BASE_ERROR_H_

#include "../variable/base_variables.h"

//Std
#include <map>
#include <set>

using namespace std;

class BaseErrorTerm
{
public:
	BaseErrorTerm(int nErrorDim)
		:m_nErrorDimension(nErrorDim)
	{
		this->m_nVariableDimension = this->GetVariableDimension();
	}

	bool AddVariable(OptimizedVariable * pVariable){
		this->m_gVariables.push_back(pVariable);
		//Change the variable of the dimension.
		this->m_nVariableDimension = this->GetVariableDimension();
	}

	OptimizedVariable * GetVariable(int nIndex){
		if (nIndex >= this->m_gVariables.size() || nIndex < 0){
			cerr << "Wrong index when finding the variable in the error term" << endl;
			return nullptr;
		}
		return this->m_gVariables[nIndex];
	}

	inline int GetVariableDimension(){
		int nDim = 0;
		for (auto pVariable : this->m_gVariables){
			nDim += pVariable->GetDimension();
		}
		return nDim;
	}

	//The error may be multi-value.
	//This is a pure virtual function.
	virtual Eigen::VectorXd ComputeError() = 0;

	//The jacobian type should be checked.
	virtual bool ComputeJacobian() = 0;

	
private:
	//Store all variables
	vector<OptimizedVariable *> m_gVariables;

	Eigen::MatrixXd m_mJacobian;

	int m_nVariableDimension;
	int m_nErrorDimension;
	
};


#endif