#ifndef POSE_LIE_H_
#define POSE_LIE_H_


#include "../../variable/base_variables.h"
#include "sophus/se3.h"
#include <iostream>
#include <vector>

using namespace std;

//Camera pose in Lie form.
class PoseLieVertex : public OptimizedVariable
{
public:
	PoseLieVertex()
		:OptimizedVariable(6)
	{

	}

	PoseLieVertex(cv::Mat mT)
		:OptimizedVariable(6)
	{
		this->SetPose(mT);
	}

	//Override the update function.
	virtual bool Update(Eigen::VectorXd mUpdate){
		//Check if the shape of the optimized variables 
		//is same to the incremental variable.
		if (mUpdate.size() != 6){
			cout << "'Wrong incremental variable for the lie algebra." << endl;
			return false;
		}
		Sophus::SE3 mIncrementalVector = Sophus::SE3::exp(mUpdate);
		Sophus::SE3 mUpdatedPose = Sophus::SE3::exp(this->m_mVariable) * mIncrementalVector;
		this->m_mVariable = mUpdatedPose.log();
		return true;	
	};


	
	//Convert matrix pose to lie pose.
	bool SetPose(cv::Mat mT){
		if (mT.rows !=4 || mT.cols !=4){
			cout << "'Wrong dimension of the pose matrix. It should be 4*4.";
			return false;
		}
		//Convert cv mat to eigen mat.
		Eigen::MatrixXd mPoseMatrix;
		cv::cv2eigen(mT, mPoseMatrix);

		//Use Lie algebra to construct the pose matrix.
		Eigen::Matrix3d mRotation = mPoseMatrix.block(0 , 0 , 3 , 3);
		Eigen::Vector3d mTranslation = mPoseMatrix.block(0 , 3 , 3 , 1);

		Sophus::SE3 mLie(mRotation, mTranslation);
		//Use the log mapping to get the 6 dimension vector.
		Eigen::VectorXd mLieEigen = mLie.log();

		return this->SetVariable(mLieEigen);
	}

	//Convert rotation and translation to lie pose.
	bool SetPose(cv::Mat mRotation, cv::Mat mTranslation){
		if (mRotation.rows !=3 || mRotation.cols !=3){
			cout << "'Wrong dimension of the rotation matrix. It should be 3*3.";
			return false;
		}

		if (mTranslation.rows !=3 || mTranslation.cols !=1){
			cout << "'Wrong dimension of the translation matrix. It should be 3*1.";
			return false;
		}

		//Convert cv mat to eigen mat.
		Eigen::Matrix3d mRotationEigen;
		Eigen::Vector3d mTranslationEigen;

		cv::cv2eigen(mRotation, mRotationEigen);
		cv::cv2eigen(mTranslation, mTranslationEigen);

		//Use Lie algebra to construct the pose matrix.
		

		Sophus::SE3 mLie(mRotationEigen, mTranslationEigen);
		//Use the log mapping to get the 6 dimension vector.
		Eigen::VectorXd mLieEigen = mLie.log();

		return this->SetVariable(mLieEigen);
	}




	//Convert Eigen matrix pose to lie pose.
	bool SetPose(Eigen::Matrix4d mPoseMatrix){
		
		//Use Lie algebra to construct the pose matrix.
		Eigen::Matrix3d mRotation = mPoseMatrix.block(0 , 0 , 3 , 3);
		Eigen::Vector3d mTranslation = mPoseMatrix.block(0 , 3 , 3 , 1);

		Sophus::SE3 mLie(mRotation, mTranslation);
		//Use the log mapping to get the 6 dimension vector.
		Eigen::VectorXd mLieEigen = mLie.log();

		return this->SetVariable(mLieEigen);
	}


	//Convert Eigen rotation and translation to lie pose.
	bool SetPose(Eigen::Matrix3d mRotation, Eigen::Vector3d mTranslation){
		
		Sophus::SE3 mLie(mRotation, mTranslation);
		//Use the log mapping to get the 6 dimension vector.
		Eigen::VectorXd mLieEigen = mLie.log();

		return this->SetVariable(mLieEigen);
	}

	Eigen::Matrix4d GetPoseMatrix(){
		Eigen::Matrix4d mPoseMatrix = Sophus::SE3::exp(this->m_mVariable).matrix();
		return mPoseMatrix;
	}
};



#endif