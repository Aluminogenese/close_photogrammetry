#include "Resection.h"

void Resection::_getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor)
{
	std::ifstream file(filePath);
	if (!file.is_open()) {
		std::cout << "Error opening file: " << filePath << std::endl;
	}
	else {
		int ID;
		double x, y;
		std::string line;
		std::getline(file, line);// 读取首行
		while (std::getline(file, line)) {
			std::istringstream iss(line);
			iss >> ID >> x >> y;
			if (ID >= 100)// ID大于100为控制点
			{
				imageCoor[ID] = Eigen::Vector2d(x, h - 1 - y);
			}
		}
		file.close();
	}
}

void Resection::_getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor)
{
	std::ifstream file(filePath);
	if (!file.is_open()) {
		std::cout << "Error opening file: " << filePath << std::endl;
	}
	else {
		int ID, num;
		double x, y, z;
		std::string line;
		file >> num;
		for (int i = 0; i < num; i++) {
			std::getline(file, line);
			std::istringstream iss(line);
			iss >> ID >> x >> y >> z;
			objCoor[ID] = Eigen::Vector3d(x, y, z);
		}
		file.close();
	}
}

void Resection::setEOP(double* EOP)
{
	memcpy(this->ext_elements, EOP, 6 * sizeof(double));
}

void Resection::setIOP(double* IOP)
{
	memcpy(this->int_elements, IOP, 3 * sizeof(double));
}

void Resection::setCoor(const std::string& imgPath, const std::string& objPath)
{
	_getImageCoor(imgPath, this->imageCoor);
	_getObjCoor(objPath, this->objCoor);
	leftHand2RightHand(this->objCoor);
}

void Resection::calculate(const std::string& filePath)
{
	//控制点个数
	int num = this->imageCoor.size();
	//计数变量
	int count = 0, i = 0;
	//外方位线元素
	double Xs = this->ext_elements[0], Ys = this->ext_elements[1], Zs = this->ext_elements[2];
	//外方位角元素
	double phi = this->ext_elements[3], omega = this->ext_elements[4], kappa = this->ext_elements[5];
	//内方位元素
	double f = this->int_elements[0], x0 = this->int_elements[1], y0 = this->int_elements[2];
	//物方点的物方坐标
	double X, Y, Z;
	//物方点在像空间坐标系中的坐标
	double Xbar, Ybar, Zbar;
	//像点坐标
	double x, y;
	//像方的改正坐标
	double dx, dy;
	// r = 径向
	double r;
	//畸变系数
	double k1 = this->distort_param[0], k2 = this->distort_param[1];
	double p1 = this->distort_param[2], p2 = this->distort_param[3];


	//旋转矩阵元素
	double a1, a2, a3;
	double b1, b2, b3;
	double c1, c2, c3;

	// 法方程系数阵
	Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(2 * num, 13);
	Eigen::MatrixXd matX = Eigen::MatrixXd::Zero(13, 1);	// Xs Ys Zs phi omega kappa f x0 y0 k1 k2 p1 p2
	Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(2 * num, 1);

	//输出结果
	std::ofstream outfile(filePath, std::ios::trunc);  //输出流
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
		return;
	}
	outfile << "======== 迭代改正数 ========" << std::endl;
	outfile << "[dXs(mm) dYs(mm) dZs(mm) dPhi(rad) dOmega(rad) dKappa(rad) df(pix) dx0(pix) dy0(pix) dk1(pix^-2) dk2(pix^-4) dp1(pix^-1) dp2(pix^-1)]" << std::endl;

	for (i = 0; i < ITER_TIMES; i++)
	{

		a1 = cos(phi) * cos(kappa) - sin(phi) * sin(omega) * sin(kappa);
		a2 = -cos(phi) * sin(kappa) - sin(phi) * sin(omega) * cos(kappa);
		a3 = -sin(phi) * cos(omega);
		b1 = cos(omega) * sin(kappa);
		b2 = cos(omega) * cos(kappa);
		b3 = -sin(omega);
		c1 = sin(phi) * cos(kappa) + cos(phi) * sin(omega) * sin(kappa);
		c2 = -sin(phi) * sin(kappa) + cos(phi) * sin(omega) * cos(kappa);
		c3 = cos(phi) * cos(omega);



		count = 0;
		//对每个像点列方程
		for (auto it = this->imageCoor.begin(); it != imageCoor.end(); it++)
		{
			x = it->second.x();
			y = it->second.y();
			// 对每一个像点搜索它的物方坐标
			X = this->objCoor[it->first].x();
			Y = this->objCoor[it->first].y();
			Z = this->objCoor[it->first].z();

			Xbar = a1 * (X - Xs) + b1 * (Y - Ys) + c1 * (Z - Zs);
			Ybar = a2 * (X - Xs) + b2 * (Y - Ys) + c2 * (Z - Zs);
			Zbar = a3 * (X - Xs) + b3 * (Y - Ys) + c3 * (Z - Zs);

			r = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
			dx = (x - x0) * (k1 * pow(r, 2) + k2 * pow(r, 4)) + p1 * (pow(r, 2) + 2 * pow(x - x0, 2)) + 2 * p2 * (x - x0) * (y - y0);
			dy = (y - y0) * (k1 * pow(r, 2) + k2 * pow(r, 4)) + p2 * (pow(r, 2) + 2 * pow(y - y0, 2)) + 2 * p1 * (x - x0) * (y - y0);


			matL(2 * count, 0) = x - (x0 - f * Xbar / Zbar - dx);
			matL(2 * count + 1, 0) = y - (y0 - f * Ybar / Zbar - dy);

			//外方位元素
			matA(2 * count, 0) = (1 / Zbar) * (a1 * f + a3 * (x - x0));
			matA(2 * count, 1) = (1 / Zbar) * (b1 * f + b3 * (x - x0));
			matA(2 * count, 2) = (1 / Zbar) * (c1 * f + c3 * (x - x0));
			matA(2 * count, 3) = (y - y0) * sin(omega) - (
				(x - x0) / f * (
					(x - x0) * cos(kappa) - (y - y0) * sin(kappa)
					) + f * cos(kappa)
				) * cos(omega);
			matA(2 * count, 4) = -f * sin(kappa) - (x - x0) / f * (
				(x - x0) * sin(kappa) + (y - y0) * cos(kappa)
				);
			matA(2 * count, 5) = y - y0;
			//内方位元素
			matA(2 * count, 6) = (x - x0) / f;
			matA(2 * count, 7) = 1;
			matA(2 * count, 8) = 0;
			//畸变系数
			matA(2 * count, 9) = -(x - x0) * pow(r, 2);
			matA(2 * count, 10) = -(x - x0) * pow(r, 4);
			matA(2 * count, 11) = -(pow(r, 2) + 2 * pow(x - x0, 2));
			matA(2 * count, 12) = -2 * (x - x0) * (y - y0);

			//外方位元素
			matA(2 * count + 1, 0) = (1 / Zbar) * (a2 * f + a3 * (y - y0));
			matA(2 * count + 1, 1) = (1 / Zbar) * (b2 * f + b3 * (y - y0));
			matA(2 * count + 1, 2) = (1 / Zbar) * (c2 * f + c3 * (y - y0));
			matA(2 * count + 1, 3) = -(x - x0) * sin(omega) - (
				(y - y0) / f * (
					(x - x0) * cos(kappa) - (y - y0) * sin(kappa)
					) - f * sin(kappa)
				) * cos(omega);
			matA(2 * count + 1, 4) = -f * cos(kappa) - (y - y0) / f * (
				(x - x0) * sin(kappa) + (y - y0) * cos(kappa)
				);
			matA(2 * count + 1, 5) = -(x - x0);
			//内方位元素
			matA(2 * count + 1, 6) = (y - y0) / f;
			matA(2 * count + 1, 7) = 0;
			matA(2 * count + 1, 8) = 1;
			//畸变系数
			matA(2 * count + 1, 9) = -(y - y0) * pow(r, 2);
			matA(2 * count + 1, 10) = -(y - y0) * pow(r, 4);
			matA(2 * count + 1, 11) = -2 * (x - x0) * (y - y0);
			matA(2 * count + 1, 12) = -(pow(r, 2) + 2 * pow(y - y0, 2));

			count++;
		}
		// 计算改正数
		matX = (matA.transpose() * matA).inverse() * matA.transpose() * matL;
		outfile << "---- 第" << i + 1 << "次迭代 ----" << std::endl;
		outfile << matX << std::endl;
		if (fabs(matX(3, 0)) < 1e-10)
		{
			break;
		}
		//更新未知数 Xs Ys Zs phi omega kappa f x0 y0 k1 k2 p1 p2
		Xs += matX(0, 0);
		Ys += matX(1, 0);
		Zs += matX(2, 0);
		phi += matX(3, 0);
		omega += matX(4, 0);
		kappa += matX(5, 0);
		f += matX(6, 0);
		x0 += matX(7, 0);
		y0 += matX(8, 0);
		k1 += matX(9, 0);
		k2 += matX(10, 0);
		p1 += matX(11, 0);
		p2 += matX(12, 0);
	}


	if (i < ITER_TIMES)
	{
		outfile << "======== 迭代成功！" << "共迭代" << i + 1 << "次 ========" << std::endl;
		outfile << "参与平差的控制点个数:" << count << std::endl;
		Eigen::MatrixXd matV = matA * matX - matL;
		outfile << "像点残差(pix):" << std::endl;
		int i = 0;
		for (auto it = this->imageCoor.begin(); it != imageCoor.end(); it++, i++)
		{
			outfile << it->first << " vx: " << matV(i * 2, 0) << " vy: " << matV(i * 2 + 1, 0) << std::endl;
		}
		outfile << "======== 精度统计 ========" << std::endl;
		// 单位权中误差
		int r = num * 2 - 13;
		double sigma = sqrt((matV.transpose() * matV)(0, 0) / r);
		outfile << "单位权中误差(pix)：" << sigma << std::endl;
		//协因数阵
		Eigen::MatrixXd matQxhxh = (matA.transpose() * matA).inverse();
		//未知数中误差
		Eigen::MatrixXd matMxhxh = matQxhxh.diagonal().cwiseSqrt() * sigma;
		//outfile << "diag(Qxhxh)" << std::endl << matQxhxh.diagonal() << std::endl;
		outfile << "未知数中误差：" << std::endl;
		outfile << "Xs(mm)：" << matMxhxh(0, 0) << std::endl;
		outfile << "Ys(mm)：" << matMxhxh(1, 0) << std::endl;
		outfile << "Zs(mm)：" << matMxhxh(2, 0) << std::endl;
		outfile << "phi(rad)：" << matMxhxh(3, 0) << std::endl;
		outfile << "omega(rad)：" << matMxhxh(4, 0) << std::endl;
		outfile << "kappa(rad)：" << matMxhxh(5, 0) << std::endl;
		outfile << "f(pix)：" << matMxhxh(6, 0) << std::endl;
		outfile << "x0(pix)：" << matMxhxh(7, 0) << std::endl;
		outfile << "y0(pix)：" << matMxhxh(8, 0) << std::endl;
		outfile << "k1(pix^-2)：" << matMxhxh(9, 0) << std::endl;
		outfile << "k2(pix^-4)：" << matMxhxh(10, 0) << std::endl;
		outfile << "p1(pix^-1)：" << matMxhxh(11, 0) << std::endl;
		outfile << "p2(pix^-1)：" << matMxhxh(12, 0) << std::endl;

		//像点最或是值
		outfile << "======== 像点最或是值(pix) ========" << std::endl;
		i = 0;
		for (auto it = this->imageCoor.begin(); it != imageCoor.end(); it++, i++)
		{
			outfile << it->first << " " << matV(i * 2, 0) + it->second.x() << " " << matV(i * 2 + 1, 0) + it->second.y() << std::endl;
		}

		this->ext_elements[0] = Xs, this->ext_elements[1] = Ys, this->ext_elements[2] = Zs;
		this->ext_elements[3] = phi, this->ext_elements[4] = omega, this->ext_elements[5] = kappa;
		this->int_elements[0] = f, this->int_elements[1] = x0, this->int_elements[2] = y0;
		this->distort_param[0] = k1, this->distort_param[1] = k2;
		this->distort_param[2] = p1, this->distort_param[3] = p2;

	}
	else
	{
		std::cout << "迭代失败！" << "共迭代" << i + 1 << "次" << std::endl;
	}

	outfile.close();
}

void Resection::saveResult(const std::string& filePath)
{
	std::ofstream outfile(filePath, std::ios::trunc);
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
	}
	double lXsYxZx[3] = { 0 };

	lXsYxZx[0] = -this->ext_elements[2];
	lXsYxZx[1] = this->ext_elements[0];
	lXsYxZx[2] = this->ext_elements[1];

	outfile << "---- 外方位线元素(mm) ----" << std::endl;
	outfile << "Xs " << lXsYxZx[0] << std::endl;
	outfile << "Ys " << lXsYxZx[1] << std::endl;
	outfile << "Zs " << lXsYxZx[2] << std::endl;

	outfile << "---- 外方位角元素(rad) ----" << std::endl;
	outfile << "phi " << this->ext_elements[3] << std::endl;
	outfile << "omega " << this->ext_elements[4] << std::endl;
	outfile << "kappa " << this->ext_elements[5] << std::endl;

	outfile << "---- 内方位元素(pix) ----" << std::endl;
	outfile << "f " << this->int_elements[0] << std::endl;
	outfile << "x0 " << this->int_elements[1] << std::endl;
	outfile << "y0 " << h - 1 - this->int_elements[2] << std::endl;

	outfile << "---- 畸变系数 ----" << std::endl;
	outfile << "k1(pix^-2) " << this->distort_param[0] << std::endl;
	outfile << "k2(pix^-4) " << this->distort_param[1] << std::endl;
	outfile << "p1(pix^-1) " << this->distort_param[2] << std::endl;
	outfile << "p2(pix^-1) " << this->distort_param[3] << std::endl;
	outfile << std::endl;

	outfile.close();
}


