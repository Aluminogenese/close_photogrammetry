#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
#include <fstream>
#include "BaseClass.h"

#define ITER_TIMES 100
class DLT
{
public:
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
	double left_x0 = 0, left_y0 = 0;
	double right_x0 = 0, right_y0 = 0;

	//����ļ���
	std::string mstr;
public:
	void getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imgCtrCoor, std::map<int, Eigen::Vector2d>& imgUknCoor);
	void getChkImgCoor(const std::string& checkConfigPath);
	void getObjCoor(const std::string& filePath);
	void setStr(const std::string& str);
	void setCoor(const std::string& leftImgPath, const std::string& rightImgPath, const std::string& controlPath, const std::string& checkConfigPath);
	void initLvalue();
	void calculateLvaue();
	void correctImgCoor();
	void initUknObjCoor();
	void calculateUknObjCoor();
};

