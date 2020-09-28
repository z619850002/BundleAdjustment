#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


#include <Eigen/Core>
#include <cmath>
#include <chrono>
#include <vector>



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <opencv2/core/eigen.hpp>


using namespace std; 


void Test1(vector<cv::Point3f> gPoints){

	//Define the intrinsic matrix
	cv::Mat mK = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);


	cout << "Intrinsic matrix is: " << endl << mK << endl;
	cout << "Distortion matrix is: " << endl << mD << endl;


	cv::Mat mRotation = (cv::Mat_<double>(3 , 3) << 1.0 , 0.0  , 0.0 ,
											 0.0   , 1.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);

	cv::Mat mRotationVector;
	cv::Mat mTranslation = (cv::Mat_<double>(3 , 1) << 0.0 , 0.0  , 0.0);


	cv::Rodrigues(mRotation, mRotationVector);

	cout << "Rotation vector is: " << endl << mRotationVector << endl;


	//Project points to camera
	vector<cv::Point2f> gImagingPoints;
	cv::projectPoints(gPoints, mRotationVector, mTranslation, mK, mD, gImagingPoints);

	cout << "gImaging points includes: " << endl;
	for (auto item :gImagingPoints){
		cout << item << endl;
	}

}



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


	mTrueTranslation2 = (cv::Mat_<double>(3 , 1) << 2.0 , 0.0  , 0.0);


	cv::Rodrigues(mTrueRotation2, mTrueRotationVector2);
}



	// Optimize(	mRotation1, mTranslation1,
	// 	 		mRotation2, mTranslation2,
	// 	 		mRotation3, mTranslation3,
	// 	 		mK1, mK2, mK3,
	// 	 		gPoints,
	// 	 		gImagingPoints1,
	// 	 		gImagingPoints2,
	// 	 		gImagingPoints3);





void Optimize(	cv::Mat mRotation1, cv::Mat mTranslation1,
		 		cv::Mat mRotation2, cv::Mat mTranslation2,
		 		cv::Mat mK1, cv::Mat mK2,
		 		vector<cv::Point3f> gPoints3d,
		 		vector<cv::Point2f> gPoints2d_1,
		 		vector<cv::Point2f> gPoints2d_2){
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton * solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    //If output the debug info.
    optimizer.setVerbose(true);


    //Add intrinsics
    g2o::CameraParameters * pCameraParameter1 = new g2o::CameraParameters(
        mK1.at<double>(0 , 0) , Eigen::Vector2d(mK1.at<double>(0 , 2) , mK1.at<double>(1 , 2)) , 0
        );
    pCameraParameter1->setId(0);
    //Add parameters to the optimizer.
    //optimizer.addParameter(pCameraParameter1);
    optimizer.addParameter(pCameraParameter1);


    g2o::CameraParameters * pCameraParameter2 = new g2o::CameraParameters(
        mK2.at<double>(0 , 0) , Eigen::Vector2d(mK2.at<double>(0 , 2) , mK2.at<double>(1 , 2)) , 0
        );
    pCameraParameter2->setId(1);
    //Add parameters to the optimizer.
    //optimizer.addParameter(pCameraParameter1);
    optimizer.addParameter(pCameraParameter2);





    //Add 2 cameras
    g2o::VertexSE3Expmap * pPose1 = new g2o::VertexSE3Expmap();
    g2o::VertexSE3Expmap * pPose2 = new g2o::VertexSE3Expmap(); 
    
    //The first front camera is the identity one.
    Eigen::Matrix3d mRotation1_Eigen;
    Eigen::Vector3d mTranslation1_Eigen;

    Eigen::Matrix3d mRotation2_Eigen;
    Eigen::Vector3d mTranslation2_Eigen;

    cv::cv2eigen(mRotation1, mRotation1_Eigen);
    cv::cv2eigen(mTranslation1, mTranslation1_Eigen);


    cv::cv2eigen(mRotation2, mRotation2_Eigen);
    cv::cv2eigen(mTranslation2, mTranslation2_Eigen);



    pPose1 -> setId(0);
    pPose1 -> setEstimate( g2o::SE3Quat(
        mRotation1_Eigen,
        mTranslation1_Eigen
        ));
    //The pose of this camera should not be changed.
    // pPose1->setMarginalized(true);
    pPose1->setFixed(true);
    optimizer.addVertex(pPose1);



    pPose2 -> setId(1);
    pPose2 -> setEstimate( g2o::SE3Quat(
        mRotation2_Eigen,
        mTranslation2_Eigen
        ));
    //The pose of this camera should not be changed.
    // pPose2->setFixed(true);

    // pPose2->setMarginalized(true);
    optimizer.addVertex(pPose2);



   //The poses of those 2 cameras have been added to the optimizer.

    //Add 3d points as the vertex.

    cout << "Add point: " << gPoints3d.size() << endl;
    for (int i=0;i<gPoints3d.size();i++){
    	int nIndex = i+5;
    	int nIndexEdge1 = nIndex + gPoints3d.size();
    	int nIndexEdge2 = nIndexEdge1 + gPoints3d.size(); 

    	cv::Point3f iPoint3D = gPoints3d[i];
    	g2o::VertexSBAPointXYZ * pPoint = new g2o::VertexSBAPointXYZ();
    	pPoint->setId(nIndex);
    	// pPoint->setFixed(true);
   	 	pPoint->setEstimate(Eigen::Vector3d(iPoint3D.x ,
                                            iPoint3D.y ,
                                          	iPoint3D.z));
    	optimizer.addVertex(pPoint);


    	g2o::EdgeProjectXYZ2UV * pEdge1 = new g2o::EdgeProjectXYZ2UV();
        pEdge1->setId( nIndexEdge1 );
        //Linked with the position of 3d points.
		pEdge1->setVertex(0 , pPoint);
		pEdge1->setVertex(1 , pPose1);
		//Set measurement and camera parameters.
		pEdge1->setMeasurement(Eigen::Vector2d(
								gPoints2d_1[i].x, 
								gPoints2d_1[i].y));
		pEdge1->setParameterId(0 , 0);
		pEdge1->setInformation(Eigen::Matrix2d::Identity());
		
		optimizer.addEdge(pEdge1);




    	g2o::EdgeProjectXYZ2UV * pEdge2 = new g2o::EdgeProjectXYZ2UV();
        pEdge2->setId( nIndexEdge2 );
        //Linked with the position of 3d points.
		pEdge2->setVertex(0 , pPoint);
		pEdge2->setVertex(1 , pPose2);
		//Set measurement and camera parameters.
		pEdge2->setMeasurement(Eigen::Vector2d(
								gPoints2d_2[i].x, 
								gPoints2d_2[i].y));
		pEdge2->setParameterId(0 , 1);
		pEdge2->setInformation(Eigen::Matrix2d::Identity());
		
		optimizer.addEdge(pEdge2);





    }

    
    optimizer.initializeOptimization();
    optimizer.optimize(1000);


    Eigen::Matrix4d iTUpdated1 =  Eigen::Isometry3d(pPose1->estimate()).matrix();
    Eigen::Matrix4d iTUpdated2 =  Eigen::Isometry3d(pPose2->estimate()).matrix();



    cout << "Pose1 is: " << endl << iTUpdated1 << endl;

    cout << "Pose2 is: " << endl << iTUpdated2 << endl;




}




int main(){

	//Firstly generate 1000 points
	
	vector<cv::Point3f> gPoints;

	for (int i=-5; i<5; i++){
		for (int k=-5; k<5;k++){
			for (int j=1; j< 4; j++){
				double nX = (double)(i);
				double nY = (double)(k);
				double nZ = (double)(j)*(3.3);
				cv::Point3f iPoint(nX, nY, nZ);
				gPoints.push_back(iPoint);
			}
		}
	}


	Test1(gPoints);


	//Define 2 cameras
	cv::Mat mK1 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD1 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);


	cv::Mat mK2 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD2 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);





	//Define the pose of 2 cameras.
	cv::Mat mRotation1, mTranslation1, mRotationVector1;
	cv::Mat mRotation2, mTranslation2, mRotationVector2;

	LoadCameraPose1(mRotation1, mRotationVector1, mTranslation1);
	LoadCameraPose2(mRotation2, mRotationVector2, mTranslation2);

	vector<cv::Point2f> gImagingPoints1, gImagingPoints2, gImagingPoints3;
	cv::projectPoints(gPoints, mRotationVector1, mTranslation1, mK1, mD1, gImagingPoints1);
	cv::projectPoints(gPoints, mRotationVector2, mTranslation2, mK2, mD2, gImagingPoints2);



	// mTranslation1.at<double>(0 , 0) = mTranslation1.at<double>(0 , 0) - 0.3;

	mTranslation2.at<double>(0 , 0) = mTranslation2.at<double>(0 , 0) + 0.2;


	Optimize(	mRotation1, mTranslation1,
		 		mRotation2, mTranslation2,
		 		mK1, mK2, 
		 		gPoints,
		 		gImagingPoints1,
		 		gImagingPoints2);


	cout << "Original Poses: " << endl;
	cout << "Rotation1 is: " << endl << mRotation1 << endl;
	cout << "Rotation2 is: " << endl << mRotation2 << endl;

	cout << "Translation1 is: " << endl << mTranslation1 << endl;
	cout << "Translation2 is: " << endl << mTranslation2 << endl;



	return 0;
}




//Backup codes.
// //Add noise to the observation.

// 	//Define the optimizer
// 	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
//     std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
//     std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
//     //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
//     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
//     g2o::SparseOptimizer optimizer;
//     optimizer.setAlgorithm(solver);
//     //If output the debug info.
//     optimizer.setVerbose(true);


//     //There are mainly 3 part. Set the pose vertices,
//     // set the camera params and set the position of 
//     //3d points.

//     //Add pose vertices.
//     //There are five pose vertices.
//     g2o::VertexSE3Expmap * pPose1 = new g2o::VertexSE3Expmap();
//     g2o::VertexSE3Expmap * pPose2 = new g2o::VertexSE3Expmap(); 
    
//     //The first front camera is the identity one.
//     pPose1 -> setId(0);
//     pPose1 -> setEstimate( g2o::SE3Quat(
//         // Eigen::Matrix3d::Identity(),
//         // Eigen::Vector3d(0.0,0.0,0.0)
        
//         this->mT_F2
//         ));
//     //The pose of this camera should not be changed.
//     pPoseFVertex -> setFixed(true);
//     optimizer.addVertex(pPoseFVertex);    

   



//     //Then we should set the camera params.
//     //There are 4 cameras.    
//     g2o::CameraParameters * pCameraParameter1 = new g2o::CameraParameters(
//         this->m_K1.at<double>(0 , 0) , Eigen::Vector2d(this->m_K1.at<double>(0 , 2) , this->m_K1.at<double>(1 , 2)) , 0
//         );
//     pCameraParameter1->setId(0);
//     //Add parameters to the optimizer.
//     //optimizer.addParameter(pCameraParameter1);
//     optimizer.addParameter(pCameraParameter1);


//     g2o::CameraParameters * pCameraParameter2 = new g2o::CameraParameters(
//         this->m_K2.at<double>(0 , 0) , Eigen::Vector2d(this->m_K2.at<double>(0 , 2) , this->m_K2.at<double>(1 , 2)) , 0
//         );
//     pCameraParameter2->setId(1);
//     //Add parameters to the optimizer.
//     //optimizer.addParameter(pCameraParameter1);
//     optimizer.addParameter(pCameraParameter2);


//         //Add 3d points as the vertex.
//         g2o::VertexSBAPointXYZ * pPoint = new g2o::VertexSBAPointXYZ();
//         pPoint->setId(index);
//         pPoint->setEstimate(Eigen::Vector3d(iPair.GetPoint3D().x ,
//                                              iPair.GetPoint3D().y ,
//                                               iPair.GetPoint3D().z));
//         optimizer.addVertex(pPoint);

//         //Add 2d points as the edge.

//         //Add the edge to the front camera pose.
//         g2o::EdgeProjectXYZ2UV * pEdgeF = new g2o::EdgeProjectXYZ2UV();
//         pEdgeF->setId( indexEdge );
//         //Set the vertex of this edge.
//         //Linked with the position of 3d points.
//         pEdgeF->setVertex(0 , pPoint);
//         //Linked with the pose.
//         pEdgeF->setVertex(1 , pPoseFVertex);
//         //Set measurement and camera parameters.
//         pEdgeF->setMeasurement(Eigen::Vector2d(iPair.GetPoint2D1().x , iPair.GetPoint2D1().y));
//         pEdgeF->setParameterId(0 , 0);
//         pEdgeF->setInformation(Eigen::Matrix2d::Identity());

//         //Add robust kernel
//         this->AddRobustKernelToEdge(pEdgeF);

//         optimizer.addEdge(pEdgeF);
//         indexEdge ++ ;

//         //Add the edge to the front2 camera pose
//         g2o::EdgeProjectXYZ2UV * pEdgeF2 = new g2o::EdgeProjectXYZ2UV();
//         pEdgeF2->setId( indexEdge );
//         //Set the vertex of this edge.
//         //Linked with the position of 3d points.
//         pEdgeF2->setVertex(0 , pPoint);
//         //Linked with the pose.
//         pEdgeF2->setVertex(1 , pPoseF2Vertex);
//         //Set measurement and camera parameters.
//         pEdgeF2->setMeasurement(Eigen::Vector2d(iPair.GetPoint2D1().x , iPair.GetPoint2D1().y));
//         pEdgeF2->setParameterId(0 , 0);
//         pEdgeF2->setInformation(Eigen::Matrix2d::Identity());

//         //Add robust kernel
//         this->AddRobustKernelToEdge(pEdgeF2);

//         optimizer.addEdge(pEdgeF2);
//         indexEdge ++ ;


//         //Add the edge to the left camera pose.
//         g2o::EdgeProjectXYZ2UV * pEdgeL = new g2o::EdgeProjectXYZ2UV();
//         pEdgeL->setId( indexEdge );
//         //Set the vertex of this edge.
//         //Linked with the position of 3d points.
//         pEdgeL->setVertex(0 , pPoint);
//         //Linked with the pose.
//         pEdgeL->setVertex(1 , pPoseLVertex);
//         //Set measurement and camera parameters.
//         pEdgeL->setMeasurement(Eigen::Vector2d(iPair.GetPoint2D2().x , iPair.GetPoint2D2().y));
//         pEdgeL->setParameterId(0 , 1);
//         pEdgeL->setInformation(Eigen::Matrix2d::Identity());

//         //Add robust kernel
//         this->AddRobustKernelToEdge(pEdgeL);

//         optimizer.addEdge(pEdgeL);
//         indexEdge ++ ;

//         index++;
//     }




