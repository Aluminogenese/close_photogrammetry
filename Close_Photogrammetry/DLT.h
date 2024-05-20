#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
#define ITER_TIMES 100
class DLT
{
public:
	//左像片控制点、待定点、检查点的像点坐标(pix)
	std::map<int, Eigen::Vector2d> leftImgCtrCoor;
	std::map<int, Eigen::Vector2d> leftImgUknCoor;
	std::map<int, Eigen::Vector2d> leftImgChkCoor;
	//右像片控制点、待定点、检查点的像点坐标(pix)
	std::map<int, Eigen::Vector2d> rightImgCtrCoor;
	std::map<int, Eigen::Vector2d> rightImgUknCoor;
	std::map<int, Eigen::Vector2d> rightImgChkCoor;

	//左右像片同名待定点的改正坐标
	std::map<int, Eigen::Vector2d> leftImgCorCoor;
	std::map<int, Eigen::Vector2d> rightImgCorCoor;

	//控制点的物方坐标
	std::map<int, Eigen::Vector3d> ctrObjCoor;
	//检查点的物方坐标
	std::map<int, Eigen::Vector3d> chkObjCoor;
	//待定点的物方坐标
	std::map<int, Eigen::Vector3d> uknObjCoor;


	//L系数矩阵
	Eigen::Matrix<double, 3, 4> leftMatL;
	Eigen::Matrix<double, 3, 4> rightMatL;
	//畸变系数
	double leftDistortParam[4] = { 0 };//k1,k2,p1,p2;
	double righttDistortParam[4] = { 0 };//k1,k2,p1,p2;

	//内方位元素x0,y0(pix)
	double left_x0 = 0, mLeft_y0 = 0;
	double right_x0 = 0, mRight_y0 = 0;

	//输出文件名
	std::string mstr;
};

