#ifndef POINT3D_H_
#define POINT3D_H_

#include "../../variable/base_variables.h"

#include <iostream>
#include <vector>
using namespace std;


class Point3DVertex : public OptimizedVariable
{
public:
	Point3DVertex()
		: OptimizedVariable(3)
	{}


	Point3DVertex(cv::Point3d iPoint)
		: OptimizedVariable(3)
	{
		this->SetPoint(iPoint);
	}


	bool SetPoint(cv::Point3d iPoint){
		Eigen::Vector3d mPointVec(iPoint.x, iPoint.y, iPoint.z);
		return this->SetVariable(mPointVec);
	}
	
};


#endif