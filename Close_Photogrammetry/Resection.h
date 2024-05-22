#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>
#include "BaseClass.h"

class Resection : public BaseClass
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
	/// ���ļ���ȡ���Ƶ�������
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	/// <param name="imageCoor">���Ƶ�������</param>
	void _getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor);

	/// <summary>
	/// ���ļ���ȡ���Ƶ��﷽�ռ�����
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	/// <param name="objCoor">���Ƶ��﷽����</param>
	void _getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor);

	/// <summary>
	/// �����ⷽλԪ�س�ʼֵ
	/// </summary>
	/// <param name="EOP">Ӱ���ⷽλԪ��</param>
	void setEOP(double* EOP);

	/// <summary>
	/// �����ڷ�λԪ�س�ʼֵ
	/// </summary>
	/// <param name="IOP">�ڷ�λԪ��</param>
	void setIOP(double* IOP);

	/// <summary>
	/// ��ȡ��������﷽���� �ⲿ�ӿ�
	/// </summary>
	/// <param name="imgPath">��������ļ�·��</param>
	/// <param name="objPath">��������ļ�·��</param>
	void setCoor(const std::string& imgPath, const std::string& objPath);

	/// <summary>
	/// ��С���˼���
	/// </summary>
	/// <param name="filePath">�м�������·��</param>
	void calculate(const std::string& filePath);

	/// <summary>
	/// ������
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	void saveResult(const std::string& filePath);
};

