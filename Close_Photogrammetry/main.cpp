#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include "BaseClass.h"
#include "Resection.h"
#include "DLT.h"

#define RESECTION_DEBUG
#define DLT_DEBUG

int main() {
	std::string leftImgPath = "data/NEW_IMG_8620.JPG.dat";
	std::string rightImgPath = "data/NEW_IMG_8621.JPG.dat";
	std::string controlDataPath = "data/2024实习-近景控制场-20240515.txt";
	std::string str;
#ifdef RESECTION_DEBUG
	double pixelSize = 22.2 / 4272;
	double lEOP[6] = { 800,0,-1000, 15.0 / 180 * M_PI,0,0 };        //使用右手坐标系！
	double rEOP[6] = { 5500 - 800,0,-1300,-15.0 / 180 * M_PI,0,0 }; //使用右手坐标系！
	double IOP[3] = { 20 / pixelSize, w / 2,h / 2 };

	//======== 左像片 ========
	Resection leftImg;
	leftImg.setEOP(lEOP);
	leftImg.setIOP(IOP);
	leftImg.setCoor(leftImgPath, controlDataPath);
	str = "left";
	leftImg.calculate("output/" + str + ".rep");
	leftImg.saveResult("output/" + str + ".res");

	//======== 右像片 ========
	Resection rightImg;
	rightImg.setEOP(rEOP);
	rightImg.setIOP(IOP);
	rightImg.setCoor(rightImgPath, controlDataPath);
	str = "right";
	rightImg.calculate("output/" + str + ".rep");
	rightImg.saveResult("output/" + str + ".res");
#endif // RESECTION_DEBUG

#ifdef DLT_DEBUG
	std::string checkConfigPath = "data/checkpoint.cig";
	str = "DLT";
	DLT dlt;
	dlt.setCoor(leftImgPath, rightImgPath, controlDataPath, checkConfigPath);
	dlt.calculateLvaue();
	dlt.calculateUknObjCoor();
	//======== 计算结果 ========
	dlt.saveResult("output/" + str + ".res");
#endif // DLT_DEBUG

	return 0;
}