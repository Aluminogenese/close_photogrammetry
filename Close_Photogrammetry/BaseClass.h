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
	/// ��ȡ������� �麯�� ����ʵ��
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	/// <param name="imageCoor">�������</param>
	virtual void _getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor) {};

	/// <summary>
	/// ��ȡ�﷽���� �麯�� ����ʵ��
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	/// <param name="objCoor">�������</param>
	virtual void _getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor) {};

	/// <summary>
	/// ������ ���麯�� ����ʵ��
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	virtual void saveResult(const std::string& filePath) = 0;

	/// <summary>
	/// ����ϵת����ϵ
	/// </summary>
	/// <param name="objCoor">�﷽������</param>
	static void leftHand2RightHand(std::map<int, Eigen::Vector3d>& objCoor);

	/// <summary>
	/// ����ϵת����ϵ
	/// </summary>
	/// <param name="objCoor">�﷽������</param>
	static void rightHand2LeftHand(std::map<int, Eigen::Vector3d>& objCoor);
};
