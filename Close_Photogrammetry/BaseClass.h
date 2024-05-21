#pragma once
#include <iostream>
#include <fstream>
#include <map>
#include <eigen3/Eigen/Dense>
#define ITER_TIMES 100
#define w 4272
#define h 2848

class BaseClass
{
public:
	virtual void getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor) = 0;
	virtual void getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor) = 0;
	static void leftHand2RightHand(std::map<int, Eigen::Vector3d>& objCoor);
	static void rightHand2LeftHand(std::map<int, Eigen::Vector3d>& objCoor);
};
