#ifndef PRIOR_ERROR_H_
#define PRIOR_ERROR_H_


#include "../../error/base_error.h"
#include "../vertices/point3d.h"
#include "../vertices/pose_lie.h"

#include <iostream>
using namespace std;

class PriorError : public BaseErrorTerm
{
public:
	PriorError(PoseLieVertex * pPoseLie, Eigen::Vector6d mPriorError)
		:BaseErrorTerm(6)
	{
		this->AddVariable(pPoseLie);
		this->m_mPriorPose = mPriorError;
	}


	//The error may be multi-value.
	//This is a pure virtual function.
	virtual Eigen::VectorXd ComputeError()
	{

		PoseLieVertex * pPoseLie = (PoseLieVertex *)this->m_gVariables[0];
		//Reprojection.
		Eigen::Vector6d mError =  this->m_mPriorPose - pPoseLie->GetVariable();
		// cout << "Error is: " << endl << mError << endl;
		return mError;
	}

	//The jacobian type should be checked.
	virtual bool ComputeJacobian()
	{
		Eigen::Matrix6d mJacobian = -Eigen::Matrix6d::Identity();
		this->m_gJacobians.push_back(mJacobian);
		return true;
	}
	
private:
	Eigen::Vector6d m_mPriorPose;

	
};



#endif