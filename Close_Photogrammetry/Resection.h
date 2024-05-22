#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>
#include "BaseClass.h"

class Resection : public BaseClass
{
public:
	// 外方位元素Xs(mm) Ys(m) Zs(mm) phi(rad) omega(rad) kappa(rad)
	double ext_elements[6] = { 0 };
	// 内方位元素f x0 y0(pixel)
	double int_elements[3] = { 0 };
	// 畸变系数k1 k2 p1 p2
	double distort_param[4] = { 0 };
	// 像点坐标
	std::map<int, Eigen::Vector2d> imageCoor;
	// 物方坐标
	std::map<int, Eigen::Vector3d>objCoor;

public:
	/// <summary>
	/// 从文件读取控制点像方坐标
	/// </summary>
	/// <param name="filePath">文件路径</param>
	/// <param name="imageCoor">控制点像方坐标</param>
	void _getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor);

	/// <summary>
	/// 从文件读取控制点物方空间坐标
	/// </summary>
	/// <param name="filePath">文件路径</param>
	/// <param name="objCoor">控制点物方坐标</param>
	void _getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor);

	/// <summary>
	/// 设置外方位元素初始值
	/// </summary>
	/// <param name="EOP">影像外方位元素</param>
	void setEOP(double* EOP);

	/// <summary>
	/// 设置内方位元素初始值
	/// </summary>
	/// <param name="IOP">内方位元素</param>
	void setIOP(double* IOP);

	/// <summary>
	/// 获取像方坐标和物方坐标 外部接口
	/// </summary>
	/// <param name="imgPath">像点坐标文件路径</param>
	/// <param name="objPath">物点坐标文件路径</param>
	void setCoor(const std::string& imgPath, const std::string& objPath);

	/// <summary>
	/// 最小二乘计算
	/// </summary>
	/// <param name="filePath">中间结果保存路径</param>
	void calculate(const std::string& filePath);

	/// <summary>
	/// 保存结果
	/// </summary>
	/// <param name="filePath">文件路径</param>
	void saveResult(const std::string& filePath);
};

