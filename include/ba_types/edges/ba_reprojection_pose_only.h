#ifndef BA_REPROJECTION_POSE_ONLY
#define BA_REPROJECTION_POSE_ONLY


#include "../../error/base_error.h"
#include "../vertices/point3d.h"
#include "../vertices/pose_lie.h"

#include <iostream>
using namespace std;




class ReprojectionPoseOnlyEdge : public BaseErrorTerm
{
public:
	ReprojectionPoseOnlyEdge(PoseLieVertex * pPoseLie, Point3DVertex * pPoint3D, cv::Point2d iPoint2D, cv::Mat mK) 
		:BaseErrorTerm(2)
	{
		//Copy the intrinsic matrix.
		this->m_mK = mK.clone();
		if (mK.rows != 3 || mK.cols != 3){
			cerr << "Wrong intrinsic matrix type! " << endl;
			return;
		}
		cv::cv2eigen(mK, this->m_mEigenK);
		//Add vertices to the edge.
		this->AddVariable(pPoseLie);

		this->m_mPoint3d = pPoint3D->GetVariable();

		//Set the estimate.
		this->m_mEstimate = Eigen::Vector2d(iPoint2D.x, iPoint2D.y);
		
	}


	//The error may be multi-value.
	//This is a pure virtual function.
	virtual Eigen::VectorXd ComputeError()
	{

		PoseLieVertex * pPoseLie = (PoseLieVertex *)this->m_gVariables[0];
		//Reprojection.
		Eigen::Vector2d mProjection = (this->ProjectPoint(pPoseLie)).segment(0,2);
		Eigen::Vector2d mError =  this->m_mEstimate - mProjection;
		// cout << "Error is: " << endl << mError << endl;
		return mError;
	}

	//The jacobian type should be checked.
	virtual bool ComputeJacobian()
	{
		//Vertices.
		PoseLieVertex * pPoseLie = (PoseLieVertex *)this->m_gVariables[0];

		//Intrinsics.
		double nFx, nFy, nCx, nCy;
		nFx = this->m_mEigenK(0 , 0);
		nFy = this->m_mEigenK(1 , 1);
		nCx = this->m_mEigenK(0 , 2);
		nCx = this->m_mEigenK(1 , 2);

		//Project them.
		Eigen::Vector3d mPointCamera3D = this->GetCameraPoint3D(pPoseLie);
		Eigen::Vector3d mHomogeneuosPoint2D = this->GetPoint2DByCameraPoint(mPointCamera3D);

		double nX = mPointCamera3D(0);
		double nY = mPointCamera3D(1);
		double nZ = mPointCamera3D(2);
		
		double nU = mHomogeneuosPoint2D(0);
		double nV = mHomogeneuosPoint2D(1);

		Eigen::Vector2d mProjection = (this->ProjectPoint(pPoseLie)).segment(0,2);

		



		//Prepare for the jacobians.
		this->m_gJacobians.clear();
		this->m_gJacobians.reserve(2);
		Eigen::MatrixXd mJacobianA(2, 6);

		//Compute jacobians.
		//Jacobian to Lie camera pose.
		//The first row of the Jacobian.
		mJacobianA(0 , 0) = nFx/nZ;
		mJacobianA(0 , 1) = 0.0;
		mJacobianA(0 , 2) = -nFx*nX/(nZ*nZ);
		mJacobianA(0 , 3) = -nFx*nX*nY/(nZ*nZ);
		mJacobianA(0 , 4) = nFx + (nFx * nX * nX / (nZ * nZ));
		mJacobianA(0 , 5) = -nFx * nY / nZ;
		//The second row of the jacobian.
		mJacobianA(1 , 0) = 0.0;
		mJacobianA(1 , 1) = nFy/nZ;
		mJacobianA(1 , 2) = -nFy*nY/(nZ*nZ);
		mJacobianA(1 , 3) = -nFy-(nFy*nY*nY/(nZ*nZ));
		mJacobianA(1 , 4) = nFy*nX*nY/(nZ*nZ);
		mJacobianA(1 , 5) = nFy*nX/nZ;

		mJacobianA = -mJacobianA;
		// cout << "mJacobianA is: " << endl << mJacobianA << endl;
		// cout << "pPoseLie is: " << endl << *pPoseLie << endl;

		
		//Register jacobians.
		this->m_gJacobians.push_back(mJacobianA);

		return true;
	}
	
private:

	inline Eigen::Vector3d ProjectPoint(PoseLieVertex * pPoseLie){
		//Firstly get the point in world coordinate and the pose matrix.
		Eigen::Vector3d mPointWorld = this->m_mPoint3d;
		Sophus::SE3 mPoseSophus = Sophus::SE3::exp(pPoseLie->GetVariable());
		Eigen::Matrix4d mPose = mPoseSophus.matrix();
		//Secondly convert the world coordinate of the point to camera coordinate.
		Eigen::Vector4d mHomogeneousPointWorld(1 , 1 , 1 , 1);
		mHomogeneousPointWorld.segment(0 , 3) = mPointWorld;
		Eigen::Vector4d mHomogeneousPointCamera = mPose * mHomogeneousPointWorld;
		//Normalize.
		mHomogeneousPointCamera /= mHomogeneousPointCamera(3);
		//Project to the imaging plane.
		Eigen::Vector3d mHomogeneuosPoint2D = this->m_mEigenK * mHomogeneousPointCamera.segment(0 , 3);
		mHomogeneuosPoint2D /= mHomogeneuosPoint2D(2);

		// cout << "Point is: " << mHomogeneuosPoint2D.transpose() << endl;
		return mHomogeneuosPoint2D;
	}

	//Compute the coordinate of the point in the camera cs.
	inline Eigen::Vector3d GetCameraPoint3D(PoseLieVertex * pPoseLie){
		Eigen::Vector3d mPointWorld = this->m_mPoint3d;
		Sophus::SE3 mPoseSophus = Sophus::SE3::exp(pPoseLie->GetVariable());
		Eigen::Matrix4d mPose = mPoseSophus.matrix();
		//Secondly convert the world coordinate of the point to camera coordinate.
		Eigen::Vector4d mHomogeneousPointWorld(1 , 1 , 1 , 1);
		mHomogeneousPointWorld.segment(0, 3) = mPointWorld;
		Eigen::Vector4d mHomogeneousPointCamera = mPose * mHomogeneousPointWorld;
		//Normalize.
		mHomogeneousPointCamera /= mHomogeneousPointCamera(3);
		return mHomogeneousPointCamera.segment(0, 3);
	}

	inline Eigen::Vector3d GetPoint2DByCameraPoint(Eigen::Vector3d mPointCamera){
		Eigen::Vector3d mHomogeneuosPoint2D = this->m_mEigenK * mPointCamera;
		mHomogeneuosPoint2D /= mHomogeneuosPoint2D(2);
		return mHomogeneuosPoint2D;
	}


	//Intrinsic matrix.
	cv::Mat m_mK;
	Eigen::Matrix3d m_mEigenK;

	//Not optimized in this edge.
	Eigen::Vector3d m_mPoint3d;

	Eigen::Vector2d m_mEstimate;
	
};


#endif