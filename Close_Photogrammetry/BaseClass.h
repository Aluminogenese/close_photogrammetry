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
	/// <summary>
	/// 获取像点坐标 虚函数 子类实现
	/// </summary>
	/// <param name="filePath">文件路径</param>
	/// <param name="imageCoor">像点坐标</param>
	virtual void _getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor) {};

	/// <summary>
	/// 获取物方坐标 虚函数 子类实现
	/// </summary>
	/// <param name="filePath">文件路径</param>
	/// <param name="objCoor">像点坐标</param>
	virtual void _getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor) {};

	/// <summary>
	/// 保存结果 纯虚函数 子类实现
	/// </summary>
	/// <param name="filePath">文件路径</param>
	virtual void saveResult(const std::string& filePath) = 0;

	/// <summary>
	/// 左手系转右手系
	/// </summary>
	/// <param name="objCoor">物方点坐标</param>
	static void leftHand2RightHand(std::map<int, Eigen::Vector3d>& objCoor);

	/// <summary>
	/// 右手系转左手系
	/// </summary>
	/// <param name="objCoor">物方点坐标</param>
	static void rightHand2LeftHand(std::map<int, Eigen::Vector3d>& objCoor);
};
