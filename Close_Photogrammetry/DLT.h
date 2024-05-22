#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
#include <fstream>
#include "BaseClass.h"

#define ITER_TIMES 100
class DLT : public BaseClass
{
private:
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
	Eigen::Vector2d left_x0y0;
	Eigen::Vector2d right_x0y0;
private:
	/// <summary>
	/// 获取像点坐标
	/// </summary>
	/// <param name="filePath">文件路径</param>
	/// <param name="imgCtrCoor">控制点像点坐标</param>
	/// <param name="imgUknCoor">待定点像点坐标</param>
	void _getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imgCtrCoor, std::map<int, Eigen::Vector2d>& imgUknCoor);

	/// <summary>
	/// 获取检查点像点坐标
	/// </summary>
	/// <param name="checkConfigPath">检查点ID文件路径</param>
	void _getChkImgCoor(const std::string& checkConfigPath);

	/// <summary>
	/// 获取控制点物方坐标
	/// </summary>
	/// <param name="filePath">文件路径</param>
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

