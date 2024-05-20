#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>
#include "BaseClass.h"

#define ITER_TIMES 100
class Resection
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
	/// 设置外方位元素
	/// </summary>
	/// <param name="EOP">影像外方位元素</param>
	void setEOP(double* EOP);
	/// <summary>
	/// 设置内方位元素
	/// </summary>
	/// <param name="IOP">内方位元素</param>
	void setIOP(double* IOP);
	void setCoor(const std::string& imgPath, const std::string& objPath);
	void calculate(const std::string& filePath);
};

