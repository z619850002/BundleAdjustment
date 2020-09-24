#ifndef BASE_VARIABLES_H_
#define BASE_VARIABLES_H_

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>


#include <iostream>

using namespace std;


class OptimizedVariable
{
public:
	OptimizedVariable(int nDimension){
		this->m_mVariable = Eigen::VectorXd(nDimension);
		this->m_mVariable.setZero();
		this->m_nDimension = nDimension;
	}

	virtual bool Update(Eigen::VectorXd mUpdate){
		//Check if the shape of the optimized variables 
		//is same to the incremental variable.
		if (mUpdate.size() == this->m_mVariable.size()){
			this->m_mVariable += mUpdate;
			return true;	
		}
		return false;
	};

	//Override the output operators.
	friend ostream &operator<<( ostream & iOutput, const OptimizedVariable & iVariable )
	{
		output << iVariable.m_mVariable;
		return output;            
	}

private:
	Eigen::VectorXd m_mVariable;
	int m_nDimension;
};






#endif