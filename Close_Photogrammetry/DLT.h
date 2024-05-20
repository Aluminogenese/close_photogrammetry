#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
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
	double righttDistortParam[4] = { 0 };//k1,k2,p1,p2;

	//�ڷ�λԪ��x0,y0(pix)
	double left_x0 = 0, mLeft_y0 = 0;
	double right_x0 = 0, mRight_y0 = 0;

	//����ļ���
	std::string mstr;
};

