#include "../include/ba_types/vertices/point3d.h"
#include "../include/ba_types/vertices/pose_lie.h"
#include "../include/ba_types/edges/ba_reprojection_error.h"

using namespace std;


int main(){
	cv::Mat mPose = (cv::Mat_<double>(4 , 4) << 1.0 , 0.0 , 0.0, 3.0,
												0.0 , 1.0 , 0.0, 2.0,
												0.0 , 0.0 , 1.0, 1.0,
												0.0 , 0.0 , 0.0, 1.0);

	cv::Point3d iPoint(2 , 4 , 3);

	PoseLieVertex iVertex;
	iVertex.SetPose(mPose);
	cout << "Vertex is: " << endl << iVertex << endl;

	Point3DVertex iPointVertex;
	iPointVertex.SetPoint(iPoint);
	cout << "Point is: " << endl << iPointVertex << endl;

	Eigen::VectorXd mUpdated(6);
	mUpdated << 0.01 , -0.01 , 0.01 , -0.01 , 0.01 , -0.01;
	iVertex.Update(mUpdated);


	cout << "Updated Vertex is: " << endl << iVertex << endl;


	return 0;
}