#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
#include <fstream>
#include "BaseClass.h"

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
	double rightDistortParam[4] = { 0 };//k1,k2,p1,p2;

	//内方位元素x0,y0(pix)
	double left_x0 = 0, left_y0 = 0;
	double right_x0 = 0, right_y0 = 0;

	//输出文件名
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

