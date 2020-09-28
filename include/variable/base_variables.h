#ifndef BASE_VARIABLES_H_
#define BASE_VARIABLES_H_

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//Std
#include <iostream>
#include <string>
#include <vector>

using namespace std;


class OptimizedVariable
{
public:

	//Constructor.
	OptimizedVariable(int nDimension){
		this->m_mVariable = Eigen::VectorXd(nDimension);
		this->m_mVariable.setZero();
		this->m_nDimension = nDimension;
	}

	//Overrided constructor. Construct the variable with an Eigen Vector.
	OptimizedVariable(Eigen::VectorXd mInitializedVec){
		this->m_mVariable = mInitializedVec;
		this->m_nDimension = mInitializedVec.size();
	}

	//Overrided constructor. Construct the variable with an cv mat.
	OptimizedVariable(cv::Mat mVectorCV){
		if (mVectorCV.cols != 1){
			cerr << "Warning:Wrong shape for the opencv matrix. It must be a vector. The first column will be used in the initialization. But if there is no data in the input matrix, something wrong may happen." << endl;
			
		}
		Eigen::MatrixXd mMatEigen;
		//Convert the opencv matrix to 
		cv::cv2eigen(mVectorCV, mMatEigen);
		this->m_mVariable = mMatEigen.col(0);
		this->m_nDimension = this->m_mVariable.size();
	}


	//Update function.
	virtual bool Update(Eigen::VectorXd mUpdate){
		if (m_bFixed){
			return true;
		}
		//Check if the shape of the optimized variables 
		//is same to the incremental variable.
		if (mUpdate.size() == this->m_mVariable.size()){
			this->m_mVariable += mUpdate;
			return true;	
		}
		return false;
	};



	//Getter and Setter. It's worth mentioning that the setter will cover the original shape.
	bool SetVariable(Eigen::VectorXd mNewVariable){
		this->m_mVariable = mNewVariable;
		this->m_nDimension = mNewVariable.size();
		return true;
	}

	bool SetVariable(cv::Mat mNewVariable_CV){
		if (mNewVariable_CV.cols != 1){
			cerr << "Warning:Wrong shape for the opencv matrix. It must be a vector. The first column will be used in the initialization. But if there is no data in the input matrix, something wrong may happen." << endl;
		}
		cv::Mat mFirstColumn = mNewVariable_CV.col(0);
		cv::cv2eigen(mFirstColumn, this->m_mVariable);
		this->m_nDimension = this->m_mVariable.size();
		return true;
	}


	//Getter.
	inline Eigen::VectorXd GetVariable(){
		return this->m_mVariable;
	}

	inline int GetDimension(){
		return this->m_nDimension;
	}


	inline int GetID(){
		return this->m_nVertexID;
	}

	inline void SetID(int nNewID){
		this->m_nVertexID = nNewID;
	}


	//Override the output operators.
	friend ostream &operator<<( ostream & iOutput, const OptimizedVariable & iVariable )
	{
		iOutput << iVariable.m_mVariable;
		return iOutput;            
	}


	void SetFixed(bool bFixed){
		this->m_bFixed = bFixed;
	}


protected:
	int m_nVertexID;

	Eigen::VectorXd m_mVariable;
	int m_nDimension;

	bool m_bFixed;
};






#endif