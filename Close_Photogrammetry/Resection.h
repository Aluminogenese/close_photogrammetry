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
	// �ⷽλԪ��Xs(mm) Ys(m) Zs(mm) phi(rad) omega(rad) kappa(rad)
	double ext_elements[6] = { 0 };
	// �ڷ�λԪ��f x0 y0(pixel)
	double int_elements[3] = { 0 };
	// ����ϵ��k1 k2 p1 p2
	double distort_param[4] = { 0 };
	// �������
	std::map<int, Eigen::Vector2d> imageCoor;
	// �﷽����
	std::map<int, Eigen::Vector3d>objCoor;

public:
	/// <summary>
	/// �����ⷽλԪ��
	/// </summary>
	/// <param name="EOP">Ӱ���ⷽλԪ��</param>
	void setEOP(double* EOP);
	/// <summary>
	/// �����ڷ�λԪ��
	/// </summary>
	/// <param name="IOP">�ڷ�λԪ��</param>
	void setIOP(double* IOP);
	void setCoor(const std::string& imgPath, const std::string& objPath);
	void calculate(const std::string& filePath);
};

