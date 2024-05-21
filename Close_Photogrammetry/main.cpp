#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include "BaseClass.h"
#include "Resection.h"
#include "DLT.h"

//#define RESECTION_DEBUG
#define DLT_DEBUG

int main() {
	std::string leftImgPath = "data/NEW_IMG_8620.JPG.dat";
	std::string rightImgPath = "data/NEW_IMG_8621.JPG.dat";
	std::string controlDataPath = "data/2024实习-近景控制场-20240515.txt";
	//std::string controlDataPath = "data/2022实习-近景控制场-20220520.txt";
#ifdef RESECTION_DEBUG
	double pixelSize = 22.2 / 4272;
	double lEOP[6] = { 800,0,-1000, 15.0 / 180 * M_PI,0,0 };        //使用右手坐标系！
	double rEOP[6] = { 5500 - 800,0,-1300,-15.0 / 180 * M_PI,0,0 }; //使用右手坐标系！
	double IOP[3] = { 20 / pixelSize, w / 2,h / 2 };

	//输出结果
	std::ofstream outfile;   //输出流

	//======== 左像片 ========
	std::string str = "left";
	outfile.open("output/" + str + ".res", std::ios::trunc);
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
		return -1;
	}
	Resection leftImg;
	leftImg.setEOP(lEOP);
	leftImg.setIOP(IOP);
	leftImg.setCoor(leftImgPath, controlDataPath);

	outfile << "======== 左像片 ========" << std::endl;
	leftImg.calculate("output/" + str + ".rep");
	double lXsYxZx[3] = { 0 };

	lXsYxZx[0] = -leftImg.ext_elements[2];
	lXsYxZx[1] = leftImg.ext_elements[0];
	lXsYxZx[2] = leftImg.ext_elements[1];

	outfile << "---- 外方位线元素(mm) ----" << std::endl;
	outfile << "Xs " << lXsYxZx[0] << std::endl;
	outfile << "Ys " << lXsYxZx[1] << std::endl;
	outfile << "Zs " << lXsYxZx[2] << std::endl;

	outfile << "---- 外方位角元素(rad) ----" << std::endl;
	outfile << "phi " << leftImg.ext_elements[3] << std::endl;
	outfile << "omega " << leftImg.ext_elements[4] << std::endl;
	outfile << "kappa " << leftImg.ext_elements[5] << std::endl;

	outfile << "---- 内方位元素(pix) ----" << std::endl;
	outfile << "f " << leftImg.int_elements[0] << std::endl;
	outfile << "x0 " << leftImg.int_elements[1] << std::endl;
	outfile << "y0 " << h - 1 - leftImg.int_elements[2] << std::endl;

	outfile << "---- 畸变系数 ----" << std::endl;
	outfile << "k1(pix^-2) " << leftImg.distort_param[0] << std::endl;
	outfile << "k2(pix^-4) " << leftImg.distort_param[1] << std::endl;
	outfile << "p1(pix^-1) " << leftImg.distort_param[2] << std::endl;
	outfile << "p2(pix^-1) " << leftImg.distort_param[3] << std::endl;
	outfile << std::endl;

	outfile.close();

	//======== 右像片 ========
	str = "right";
	outfile.open("output/" + str + ".res", std::ios::trunc);
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
		return -1;
	}

	Resection rightImg;
	rightImg.setEOP(rEOP);
	rightImg.setIOP(IOP);
	rightImg.setCoor(leftImgPath, controlDataPath);

	outfile << "======== 右像片 ========" << std::endl;
	rightImg.calculate("output/" + str + ".rep");
	double rXsYxZx[3] = { 0 };

	rXsYxZx[0] = -rightImg.ext_elements[2];
	rXsYxZx[1] = rightImg.ext_elements[0];
	rXsYxZx[2] = rightImg.ext_elements[1];

	outfile << "---- 外方位线元素(mm) ----" << std::endl;
	outfile << "Xs " << rXsYxZx[0] << std::endl;
	outfile << "Ys " << rXsYxZx[1] << std::endl;
	outfile << "Zs " << rXsYxZx[2] << std::endl;

	outfile << "---- 外方位角元素(rad) ----" << std::endl;
	outfile << "phi " << rightImg.ext_elements[3] << std::endl;
	outfile << "omega " << rightImg.ext_elements[4] << std::endl;
	outfile << "kappa " << rightImg.ext_elements[5] << std::endl;

	outfile << "---- 内方位元素(pix) ----" << std::endl;
	outfile << "f " << rightImg.int_elements[0] << std::endl;
	outfile << "x0 " << rightImg.int_elements[1] << std::endl;
	outfile << "y0 " << h - 1 - rightImg.int_elements[2] << std::endl;

	outfile << "---- 畸变系数 ----" << std::endl;
	outfile << "k1(pix^-2) " << rightImg.distort_param[0] << std::endl;
	outfile << "k2(pix^-4) " << rightImg.distort_param[1] << std::endl;
	outfile << "p1(pix^-1) " << rightImg.distort_param[2] << std::endl;
	outfile << "p2(pix^-1) " << rightImg.distort_param[3] << std::endl;
	outfile << std::endl;

	outfile.close();

#endif // RESECTION_DEBUG

#ifdef DLT_DEBUG
	std::string checkConfigPath = "data/checkpoint.cig";
	std::string str = "DLT";
	DLT dlt;
	dlt.setCoor(leftImgPath, rightImgPath, controlDataPath, checkConfigPath);
	dlt.setStr(str);
	dlt.calculateLvaue();
	dlt.calculateUknObjCoor();
	//======== 计算结果 ========
	std::ofstream outfile;   //输出流
	outfile.open("output/" + str + ".res", std::ios::trunc);
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
		return -1;
	}

	std::map<int, Eigen::Vector3d> temp = dlt.uknObjCoor;
	BaseClass::rightHand2LeftHand(temp);
	auto it52 = temp.find(52);

	outfile << "======== 计算待定点坐标X,Y,Z(mm)和与52号点间的距离(mm) ========" << std::endl;
	for (auto it = temp.begin(); it != temp.end(); it++)
	{
		outfile << it->first << " " << it->second.x() << " " << it->second.y() << " " << it->second.z() << " " << sqrt(pow(it->second.x() - it52->second.x(), 2)
			+ pow(it->second.y() - it52->second.y(), 2) + pow(it->second.z() - it52->second.z(), 2)) << std::endl;
	}
	outfile.close();


#endif // DLT_DEBUG

	return 0;
}