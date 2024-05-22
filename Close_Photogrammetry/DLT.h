#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
#include <fstream>
#include "BaseClass.h"

#define ITER_TIMES 100
class DLT : public BaseClass
{
private:
	//����Ƭ���Ƶ㡢�����㡢������������(pix)
	std::map<int, Eigen::Vector2d> leftImgCtrCoor;
	std::map<int, Eigen::Vector2d> leftImgUknCoor;
	std::map<int, Eigen::Vector2d> leftImgChkCoor;
	//����Ƭ���Ƶ㡢�����㡢������������(pix)
	std::map<int, Eigen::Vector2d> rightImgCtrCoor;
	std::map<int, Eigen::Vector2d> rightImgUknCoor;
	std::map<int, Eigen::Vector2d> rightImgChkCoor;

	//������Ƭͬ��������ĸ�������
	std::map<int, Eigen::Vector2d> leftImgCorCoor;
	std::map<int, Eigen::Vector2d> rightImgCorCoor;

	//���Ƶ���﷽����
	std::map<int, Eigen::Vector3d> ctrObjCoor;
	//������﷽����
	std::map<int, Eigen::Vector3d> chkObjCoor;
	//��������﷽����
	std::map<int, Eigen::Vector3d> uknObjCoor;


	//Lϵ������
	Eigen::Matrix<double, 3, 4> leftMatL;
	Eigen::Matrix<double, 3, 4> rightMatL;
	//����ϵ��
	double leftDistortParam[4] = { 0 };//k1,k2,p1,p2;
	double rightDistortParam[4] = { 0 };//k1,k2,p1,p2;

	//�ڷ�λԪ��x0,y0(pix)
	Eigen::Vector2d left_x0y0;
	Eigen::Vector2d right_x0y0;
private:
	/// <summary>
	/// ��ȡ�������
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	/// <param name="imgCtrCoor">���Ƶ��������</param>
	/// <param name="imgUknCoor">�������������</param>
	void _getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imgCtrCoor, std::map<int, Eigen::Vector2d>& imgUknCoor);

	/// <summary>
	/// ��ȡ�����������
	/// </summary>
	/// <param name="checkConfigPath">����ID�ļ�·��</param>
	void _getChkImgCoor(const std::string& checkConfigPath);

	/// <summary>
	/// ��ȡ���Ƶ��﷽����
	/// </summary>
	/// <param name="filePath">�ļ�·��</param>
	void _getObjCoor(const std::string& filePath);

	void _initLvalue(const std::map<int, Eigen::Vector2d>& imgCtrCoor,
		const std::map<int, Eigen::Vector3d>& objCtrCoor,
		Eigen::Matrix<double, 3, 4>& MatL,
		Eigen::Vector2d& x0y0);

	void _calculateLvalue(const std::string& reportPath,
		const std::map<int, Eigen::Vector2d>& imgCtrCoor, 
		const std::map<int, Eigen::Vector3d>& objCtrCoor,
		double*distortParam,
		Eigen::Matrix<double, 3, 4>& MatL, 
		Eigen::Vector2d& x0y0);

public:
	void setCoor(const std::string& leftImgPath, const std::string& rightImgPath, const std::string& controlPath, const std::string& checkConfigPath);
	void calculateLvaue();
	void correctImgCoor();
	void initUknObjCoor();
	void calculateUknObjCoor();
	void saveResult(const std::string& filePath);

};

