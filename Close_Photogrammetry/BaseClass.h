#pragma once
#include <iostream>
#include <fstream>
#include <map>
#include <eigen3/Eigen/Dense>
class BaseClass
{
public:
	static void getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor);
	static void getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor);
	static void leftHand2RightHand(const std::map<int, Eigen::Vector3d>& origin, std::map<int, Eigen::Vector3d>& result);
	static void rightHand2LeftHand(const std::map<int, Eigen::Vector3d>& origin, std::map<int, Eigen::Vector3d>& result);
};
