#include "../include/ba_types/vertices/point3d.h"
#include "../include/ba_types/vertices/pose_lie.h"
#include "../include/ba_types/edges/ba_reprojection_error.h"
#include "../include/optimizer/optimizer.h"
using namespace std;





//Load the pose of the first camera.
void LoadCameraPose1(cv::Mat & mTrueRotation1,cv::Mat & mTrueRotationVector1, cv::Mat & mTrueTranslation1){
	mTrueRotation1 = (cv::Mat_<double>(3 , 3) << 1.0 , 0.0  , 0.0 ,
											 0.0   , 1.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);


	mTrueTranslation1 = (cv::Mat_<double>(3 , 1) << 0.0 , 0.0  , 0.0);


	cv::Rodrigues(mTrueRotation1, mTrueRotationVector1);

}


//Load the pose of the second camera.
void LoadCameraPose2(cv::Mat & mTrueRotation2,cv::Mat & mTrueRotationVector2, cv::Mat & mTrueTranslation2){
	mTrueRotation2 = (cv::Mat_<double>(3 , 3) << 1.0 , 0.0  , 0.0 ,
											 0.0   , 1.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);


	mTrueTranslation2 = (cv::Mat_<double>(3 , 1) << 3.0 , 0.0  , 0.0);


	cv::Rodrigues(mTrueRotation2, mTrueRotationVector2);
}




vector<cv::Point2d> ConvertPoints2FTo2D(vector<cv::Point2f> gPoints){
	vector<cv::Point2d> gResPoints;
	gResPoints.reserve(gPoints.size());
	for (auto iPoint : gPoints){
		gResPoints.push_back(cv::Point2d(iPoint.x, iPoint.y));
	}
	return gResPoints;
}

vector<cv::Point3d> ConvertPoints3FTo3D(vector<cv::Point3f> gPoints){
	vector<cv::Point3d> gResPoints;
	gResPoints.reserve(gPoints.size());
	for (auto iPoint : gPoints){
		gResPoints.push_back(cv::Point3d(iPoint.x, iPoint.y, iPoint.z));
	}
	return gResPoints;
}



void PreparePointsAndPoses(	cv::Mat & mPose_1, cv::Mat & mPose_2,
							vector<cv::Point2d> & gPoints2d_1,
							vector<cv::Point2d> & gPoints2d_2,
							vector<cv::Point3d> & gPoints3d,
							cv::Mat & mK1, cv::Mat & mK2){
	
	//Firstly generate 1000 points
	
	vector<cv::Point3f> gPoints;

	for (int i=-5; i<=5; i+=1){
		for (int k=-5; k<=5;k+=1){
			for (int j=1; j< 5; j+=1){
				double nX = (double)(i);
				double nY = (double)(k);
				double nZ = (double)(j)*(3.3);
				cv::Point3f iPoint(nX, nY, nZ);
				gPoints.push_back(iPoint);
			}
		}
	}


	//Define 2 cameras
	mK1 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD1 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);


	mK2 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD2 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);


	//Define the pose of 2 cameras.
	cv::Mat mRotation1, mTranslation1, mRotationVector1;
	cv::Mat mRotation2, mTranslation2, mRotationVector2;

	LoadCameraPose1(mRotation1, mRotationVector1, mTranslation1);
	LoadCameraPose2(mRotation2, mRotationVector2, mTranslation2);

	vector<cv::Point2f> gImagingPoints1, gImagingPoints2;
	cv::projectPoints(gPoints, mRotationVector1, mTranslation1, mK1, mD1, gImagingPoints1);
	cv::projectPoints(gPoints, mRotationVector2, mTranslation2, mK2, mD2, gImagingPoints2);



	// mTranslation1.at<double>(0 , 0) = mTranslation1.at<double>(0 , 0) - 0.3;

	// mTranslation2.at<double>(0 , 0) = mTranslation2.at<double>(0 , 0) + 0.2;


	//Generate Pose matrix.
	mPose_1 = (	cv::Mat_<double>(4 , 4) << 	mRotation1.at<double>(0 , 0) , mRotation1.at<double>(0 , 1) , mRotation1.at<double>(0 , 2), mTranslation1.at<double>(0 , 0),
											mRotation1.at<double>(1 , 0) , mRotation1.at<double>(1 , 1) , mRotation1.at<double>(1 , 2), mTranslation1.at<double>(1 , 0),
											mRotation1.at<double>(2 , 0) , mRotation1.at<double>(2 , 1) , mRotation1.at<double>(2 , 2), mTranslation1.at<double>(2 , 0),
											0.0 , 0.0 , 0.0, 1.0);

	mPose_2 = (	cv::Mat_<double>(4 , 4) << 	mRotation2.at<double>(0 , 0) , mRotation2.at<double>(0 , 1) , mRotation2.at<double>(0 , 2), mTranslation2.at<double>(0 , 0),
											mRotation2.at<double>(1 , 0) , mRotation2.at<double>(1 , 1) , mRotation2.at<double>(1 , 2), mTranslation2.at<double>(1 , 0),
											mRotation2.at<double>(2 , 0) , mRotation2.at<double>(2 , 1) , mRotation2.at<double>(2 , 2), mTranslation2.at<double>(2 , 0),
											0.0 , 0.0 , 0.0, 1.0);

	cout << "mPose_1 is: " << endl << mPose_1 << endl;
	cout << "mPose_2 is: " << endl << mPose_2 << endl;


	//Generate 2D points.
	gPoints2d_1 = ConvertPoints2FTo2D(gImagingPoints1);
	gPoints2d_2 = ConvertPoints2FTo2D(gImagingPoints2);

	gPoints3d = ConvertPoints3FTo3D(gPoints);

}


int main(){

	//Initialize variables.
	cv::Mat mPose_1;
	cv::Mat mPose_2;
	vector<cv::Point2d> gPoints2d_1;
	vector<cv::Point2d> gPoints2d_2;
	vector<cv::Point3d> gPoints3d;
	//Intrinsics.
	cv::Mat mK1, mK2;

	PreparePointsAndPoses(mPose_1, mPose_2, gPoints2d_1, gPoints2d_2, gPoints3d, mK1, mK2);

	cout << "Disturbed Pose1 is: " << endl << mPose_1 << endl;
	cout << "Disturbed Pose2 is: " << endl << mPose_2 << endl;


	//Create the optimizer.
	Optimizer iOptimizer;

	//Create the vertices of poses.
	PoseLieVertex * pPoseVertex1 = new PoseLieVertex(mPose_1);
	pPoseVertex1->SetID(0);
	pPoseVertex1->SetFixed(true);
	PoseLieVertex * pPoseVertex2 = new PoseLieVertex(mPose_2);
	pPoseVertex2->SetID(1);

	iOptimizer.AddVertex(pPoseVertex1);
	iOptimizer.AddVertex(pPoseVertex2);


	vector<Point3DVertex *> gVertices3D;
	vector<ReprojectionEdge *> gEdges;

	//Create edge and vertices.
	for (int i=0;i<gPoints3d.size();i++){
		int nIndex = i+5;
    	int nIndexEdge1 = nIndex + gPoints3d.size();
    	int nIndexEdge2 = nIndexEdge1 + gPoints3d.size(); 

    	//Create the vertex of 3D point.
    	cv::Point3d iPoint3d = gPoints3d[i];
    	Point3DVertex * pPointVertex = new Point3DVertex(iPoint3d);
    	pPointVertex->SetID(nIndex);
    	pPointVertex->SetFixed(true);
    	iOptimizer.AddVertex(pPointVertex);
    	gVertices3D.push_back(pPointVertex);

    	//Create edges.
    	//Jacobian to the first camera.
    	ReprojectionEdge * pEdge1 = new ReprojectionEdge(pPoseVertex1,
    			 										 pPointVertex, 
    			 										 gPoints2d_1[i], 
    			 										 mK1);
    	gEdges.push_back(pEdge1);
    	pEdge1->SetID(nIndexEdge1);
    	iOptimizer.AddEdge(pEdge1);

    	//Jacobian to the second camera.
    	ReprojectionEdge * pEdge2 = new ReprojectionEdge(pPoseVertex2,
    			 										 pPointVertex, 
    			 										 gPoints2d_2[i], 
    			 										 mK2);
    	pEdge2->SetID(nIndexEdge2);
    	iOptimizer.AddEdge(pEdge2);

	}

	// cout << "Point size is: " << gPoints3d.size() << endl;
	iOptimizer.InitializeOptimziation();


	// for (auto item : gVertices3D){
	// 	Block2D iBlock = *iOptimizer.GetBlock(gEdges[0], item);
	// 	cout << "Block is: " << endl << iBlock.m_nStartRow << " " << iBlock.m_nStartCol << " " << iBlock.m_nRows << " " << iBlock.m_nCols << " " << endl;
	// }




	for (int i=0;i<1;i++){
		iOptimizer.Optimize(3);
		cout << "After optimization, the poses are: " << endl;
		cout << pPoseVertex1->GetPoseMatrix() << endl;
		cout << pPoseVertex2->GetPoseMatrix() << endl;	
	}
	

	return 0;
}