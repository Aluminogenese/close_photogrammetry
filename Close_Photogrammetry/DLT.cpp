#include "DLT.h"

void DLT::getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imgCtrCoor, std::map<int, Eigen::Vector2d>& imgUknCoor)
{
    //先读控制点和待定点
    std::ifstream file(filePath, std::ios::in);
    if (!file.is_open())
    {
        std::cout << "right image coordinate file not found." << std::endl;
    }
    else {
        int ID;
        double x, y;
        std::string line;
        std::getline(file, line);		//跳过注释信息
        while (std::getline(file, line))           // 若未到文件结束一直循环
        {
            std::istringstream iss(line);

            iss >> ID >> x >> y;
            // 像点坐标量测时是以左上角为原点的，要转换为以左下角为原点的坐标
            if (ID >= 100)
            {
                imgCtrCoor[ID] = Eigen::Vector2d(x, h - 1 - y);
            }
            else
            {
                imgUknCoor[ID] = Eigen::Vector2d(x, h - 1 - y);
            }
        }
        file.close();   //关闭文件
    }

}

void DLT::getChkImgCoor(const std::string& checkConfigPath)
{
    std::ifstream file(checkConfigPath, std::ios::in);
    if (!file.is_open())
    {
        std::cout << "check point config file not found." << std::endl;
    }
    else {
        int ID, num;
        file >> num;
        for (int i = 0; i < num; i++)
        {
            file >> ID;
            if (ID >= 100)
            {
                auto it1 = this->leftImgCtrCoor.find(ID);
                auto it2 = this->rightImgCtrCoor.find(ID);
                if (it1 != this->leftImgCtrCoor.end() && it2 != this->rightImgCtrCoor.end())
                {
                    this->leftImgChkCoor[ID] = it1->second;
                    this->rightImgChkCoor[ID] = it2->second;
                    //移出控制点
                    this->leftImgCtrCoor.erase(it1);
                    this->rightImgCtrCoor.erase(it2);
                }
            }
        }
        file.close();   //关闭文件
    }

}

void DLT::getObjCoor(const std::string& filePath)
{
    std::ifstream file(filePath, std::ios::in);
    if (!file.is_open())
    {
        std::cout << "control point coordinate file not found." << std::endl;
    }
    else {
        int ID, num;
        double x, y, z;
        std::string line;
        file >> num;
        for (int i = 0; i < num; i++)
        {
            std::getline(file, line);
            std::istringstream iss(line);

            iss >> ID >> x >> y >> z;
            this->ctrObjCoor[ID] = Eigen::Vector3d(x, y, z);
        }
        file.close();   //关闭文件
    }
    //选检查点
    for (auto it = this->leftImgChkCoor.begin(); it != this->leftImgChkCoor.end(); it++)
    {
        auto it1 = this->ctrObjCoor.find(it->first);
        this->chkObjCoor[it->first] = it1->second;
        this->ctrObjCoor.erase(it1);
    }

}

void DLT::setStr(const std::string& str)
{
	this->mstr = str;
}

void DLT::setCoor(const std::string& leftImgPath, const std::string& rightImgPath, const std::string& controlPath, const std::string& checkConfigPath)
{
    getImageCoor(leftImgPath, this->leftImgCtrCoor, this->leftImgUknCoor);
    getImageCoor(rightImgPath, this->rightImgCtrCoor, this->rightImgUknCoor);
    getChkImgCoor(checkConfigPath);
    getObjCoor(controlPath);
	BaseClass::leftHand2RightHand(this->ctrObjCoor);
	BaseClass::leftHand2RightHand(this->chkObjCoor);
}

void DLT::initLvalue()
{
	//Eigen::MatrixXd matA();
	int ID;
	double X = 0, Y = 0, Z = 0;
	double x = 0, y = 0;
	int num = 0;
	int i = 0;//计数变量
	double L1, L2, L3, L4;
	double L5, L6, L7, L8;
	double L9, L10, L11;
	//左像片近似值计算
	{
		//计算L系数
		num = this->leftImgCtrCoor.size();
		Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(2 * num, 11);
		Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(2 * num, 1);
		i = 0;
		for (auto it = this->leftImgCtrCoor.begin(); it != this->leftImgCtrCoor.end(); it++)
		{
			ID = it->first;
			x = it->second.x();
			y = it->second.y();
			X = this->ctrObjCoor.at(ID).x();
			Y = this->ctrObjCoor.at(ID).y();
			Z = this->ctrObjCoor.at(ID).z();

			matA(i * 2, 0) = X;
			matA(i * 2, 1) = Y;
			matA(i * 2, 2) = Z;
			matA(i * 2, 3) = 1;
			matA(i * 2, 4) = 0;
			matA(i * 2, 5) = 0;
			matA(i * 2, 6) = 0;
			matA(i * 2, 7) = 0;
			matA(i * 2, 8) = x * X;
			matA(i * 2, 9) = x * Y;
			matA(i * 2, 10) = x * Z;

			matA(i * 2 + 1, 0) = 0;
			matA(i * 2 + 1, 1) = 0;
			matA(i * 2 + 1, 2) = 0;
			matA(i * 2 + 1, 3) = 0;
			matA(i * 2 + 1, 4) = X;
			matA(i * 2 + 1, 5) = Y;
			matA(i * 2 + 1, 6) = Z;
			matA(i * 2 + 1, 7) = 1;
			matA(i * 2 + 1, 8) = y * X;
			matA(i * 2 + 1, 9) = y * Y;
			matA(i * 2 + 1, 10) = y * Z;

			matL(i * 2, 0) = -x;
			matL(i * 2 + 1, 0) = -y;

			i++;
		}
		Eigen::MatrixXd temp = (matA.transpose() * matA).inverse() * matA.transpose() * matL;
		L1 = this->leftMatL(0, 0) = temp(0, 0); L2 = this->leftMatL(0, 1) = temp(1, 0); L3 = this->leftMatL(0, 2) = temp(2, 0); L4 = this->leftMatL(0, 3) = temp(3, 0);
		L5 = this->leftMatL(1, 0) = temp(4, 0); L6 = this->leftMatL(1, 1) = temp(5, 0); L7 = this->leftMatL(1, 2) = temp(6, 0); L8 = this->leftMatL(1, 3) = temp(7, 0);
		L9 = this->leftMatL(2, 0) = temp(8, 0); L10 = this->leftMatL(2, 1) = temp(9, 0); L11 = this->leftMatL(2, 2) = temp(10, 0); this->leftMatL(2, 3) = 1;

		//计算内方位元素x0,y0
		this->left_x0 = -(L1 * L9 + L2 * L10 + L3 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);
		this->left_y0 = -(L5 * L9 + L6 * L10 + L7 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);

	}
	//右像片近似值计算
	{
		//计算L系数
		num = this->rightImgCtrCoor.size();
		Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(2 * num, 11);
		Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(2 * num, 1);
		i = 0;
		for (auto it = this->rightImgCtrCoor.begin(); it != rightImgCtrCoor.end(); it++)
		{
			ID = it->first;
			x = it->second.x();
			y = it->second.y();
			X = this->ctrObjCoor.at(ID).x();
			Y = this->ctrObjCoor.at(ID).y();
			Z = this->ctrObjCoor.at(ID).z();

			matA(i * 2, 0) = X;
			matA(i * 2, 1) = Y;
			matA(i * 2, 2) = Z;
			matA(i * 2, 3) = 1;
			matA(i * 2, 4) = 0;
			matA(i * 2, 5) = 0;
			matA(i * 2, 6) = 0;
			matA(i * 2, 7) = 0;
			matA(i * 2, 8) = x * X;
			matA(i * 2, 9) = x * Y;
			matA(i * 2, 10) = x * Z;

			matA(i * 2 + 1, 0) = 0;
			matA(i * 2 + 1, 1) = 0;
			matA(i * 2 + 1, 2) = 0;
			matA(i * 2 + 1, 3) = 0;
			matA(i * 2 + 1, 4) = X;
			matA(i * 2 + 1, 5) = Y;
			matA(i * 2 + 1, 6) = Z;
			matA(i * 2 + 1, 7) = 1;
			matA(i * 2 + 1, 8) = y * X;
			matA(i * 2 + 1, 9) = y * Y;
			matA(i * 2 + 1, 10) = y * Z;

			matL(i * 2, 0) = -x;
			matL(i * 2 + 1, 0) = -y;

			i++;
		}
		Eigen::MatrixXd temp = (matA.transpose() * matA).inverse() * matA.transpose() * matL;
		L1 = this->rightMatL(0, 0) = temp(0, 0); L2 = this->rightMatL(0, 1) = temp(1, 0); L3 = this->rightMatL(0, 2) = temp(2, 0); L4 = this->rightMatL(0, 3) = temp(3, 0);
		L5 = this->rightMatL(1, 0) = temp(4, 0); L6 = this->rightMatL(1, 1) = temp(5, 0); L7 = this->rightMatL(1, 2) = temp(6, 0); L8 = this->rightMatL(1, 3) = temp(7, 0);
		L9 = this->rightMatL(2, 0) = temp(8, 0); L10 = this->rightMatL(2, 1) = temp(9, 0); L11 = this->rightMatL(2, 2) = temp(10, 0); this->rightMatL(2, 3) = 1;

		//计算内方位元素x0,y0
		this->right_x0 = -(L1 * L9 + L2 * L10 + L3 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);
		this->right_y0 = -(L5 * L9 + L6 * L10 + L7 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);

	}

}

void DLT::calculateLvaue()
{
	//计算L系数和x0,y0初始值
	initLvalue();

	std::ofstream outfile;   //输出流

	//---- 左像片“后方交会” ----
	{
		outfile.open("output/" + this->mstr + "_LeftL.rep", std::ios::trunc);
		if (!outfile.is_open())
		{
			std::cout << "output failed" << std::endl;
			return;
		}

		//控制点个数
		int num = this->leftImgCtrCoor.size();
		//计数变量
		int count = 0, i = 0;
		//内方位元素
		double x0 = this->left_x0, y0 = this->left_y0;
		//物方点的物方坐标
		double X, Y, Z;
		//像点坐标
		double x, y;
		//L系数
		double L1, L2, L3, L4;
		double L5, L6, L7, L8;
		double L9, L10, L11;
		//分母A
		double A = 1;
		// r = 径向
		double r;
		//畸变系数
		double k1 = this->leftDistortParam[0], k2 = this->leftDistortParam[1];
		double p1 = this->leftDistortParam[0], p2 = this->leftDistortParam[1];
		//法方程系数阵
		Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(2 * num, 15);
		Eigen::MatrixXd matX = Eigen::MatrixXd::Zero(15, 1);	//L1 ... L11 k1 k2 p1 p2
		//法方程常数项
		Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(2 * num, 1);
		//记录上一次迭代的fx的数值(pix)
		double tmpOld, tmpNew;
		double tmpA;
		double tmpB;
		double tmpC;
		double gamma3sq;

		x0 = this->left_x0, y0 = this->left_y0;
		L1 = this->leftMatL(0, 0), L2 = this->leftMatL(0, 1), L3 = this->leftMatL(0, 2), L4 = this->leftMatL(0, 3);
		L5 = this->leftMatL(1, 0), L6 = this->leftMatL(1, 1), L7 = this->leftMatL(1, 2), L8 = this->leftMatL(1, 3);
		L9 = this->leftMatL(2, 0), L10 = this->leftMatL(2, 1), L11 = this->leftMatL(2, 2);

		outfile << "======== 初值计算结果 ========" << std::endl;
		outfile << "L系数" << std::endl;
		outfile << L1 << " " << L2 << " " << L3 << " " << L4 << " " << std::endl;
		outfile << L5 << " " << L6 << " " << L7 << " " << L8 << " " << std::endl;
		outfile << L9 << " " << L10 << " " << L11 << " " << std::endl;
		outfile << "畸变参数" << std::endl;
		outfile << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " " << std::endl;
		outfile << "像主点坐标(pix)" << std::endl;
		outfile << "x0 " << x0 << " y0 " << y0 << " " << std::endl;

		gamma3sq = 1 / (L9 * L9 + L10 * L10 + L11 * L11);
		tmpA = gamma3sq * (L1 * L1 + L2 * L2 + L3 * L3) - x0 * x0;
		tmpB = gamma3sq * (L5 * L5 + L6 * L6 + L7 * L7) - y0 * y0;
		tmpC = gamma3sq * (L1 * L5 + L2 * L6 + L3 * L7) - x0 * y0;
		tmpOld = sqrt((tmpA * tmpB - tmpC * tmpC) / tmpB);

		outfile << "======== 迭代计算结果 ========" << std::endl;
		for (i = 0; i < ITER_TIMES; i++)
		{
			count = 0;
			//对每个像点列方程
			for (auto it = this->leftImgCtrCoor.begin(); it != leftImgCtrCoor.end(); it++)
			{
				x = it->second.x();
				y = it->second.y();
				// 对每一个像点搜索它的物方坐标
				X = this->ctrObjCoor.at(it->first).x();
				Y = this->ctrObjCoor.at(it->first).y();
				Z = this->ctrObjCoor.at(it->first).z();

				r = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
				A = L9 * X + L10 * Y + L11 * Z + 1;


				matL(2 * count, 0) = x / A;
				matL(2 * count + 1, 0) = y / A;

				matA(2 * count, 0) = X / A;
				matA(2 * count, 1) = Y / A;
				matA(2 * count, 2) = Z / A;
				matA(2 * count, 3) = 1 / A;
				matA(2 * count, 4) = 0;
				matA(2 * count, 5) = 0;
				matA(2 * count, 6) = 0;
				matA(2 * count, 7) = 0;
				matA(2 * count, 8) = x * X / A;
				matA(2 * count, 9) = x * Y / A;
				matA(2 * count, 10) = x * Z / A;
				matA(2 * count, 11) = (x - x0) * pow(r, 2);
				matA(2 * count, 12) = (x - x0) * pow(r, 4);
				matA(2 * count, 13) = pow(r, 2) + 2 * pow(x - x0, 2);
				matA(2 * count, 14) = 2 * (x - x0) * (y - y0);

				matA(2 * count + 1, 0) = 0;
				matA(2 * count + 1, 1) = 0;
				matA(2 * count + 1, 2) = 0;
				matA(2 * count + 1, 3) = 0;
				matA(2 * count + 1, 4) = X / A;
				matA(2 * count + 1, 5) = Y / A;
				matA(2 * count + 1, 6) = Z / A;
				matA(2 * count + 1, 7) = 1 / A;
				matA(2 * count + 1, 8) = y * X / A;
				matA(2 * count + 1, 9) = y * Y / A;
				matA(2 * count + 1, 10) = y * Z / A;
				matA(2 * count + 1, 11) = (y - y0) * pow(r, 2);
				matA(2 * count + 1, 12) = (y - y0) * pow(r, 4);
				matA(2 * count + 1, 13) = 2 * (x - x0) * (y - y0);
				matA(2 * count + 1, 14) = pow(r, 2) + 2 * pow(y - y0, 2);

				count++;
			}
			matA = -matA;
			//计算新值
			matX = (matA.transpose() * matA).inverse() * matA.transpose() * matL;

			//更新未知数 L1 ... L11 k1 k2 p1 p2 
			L1 = matX(0, 0);
			L2 = matX(1, 0);
			L3 = matX(2, 0);
			L4 = matX(3, 0);
			L5 = matX(4, 0);
			L6 = matX(5, 0);
			L7 = matX(6, 0);
			L8 = matX(7, 0);
			L9 = matX(8, 0);
			L10 = matX(9, 0);
			L11 = matX(10, 0);
			k1 = matX(11, 0);
			k2 = matX(12, 0);
			p1 = matX(13, 0);
			p2 = matX(14, 0);
			//更新内方位元素
			x0 = -(L1 * L9 + L2 * L10 + L3 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);
			y0 = -(L5 * L9 + L6 * L10 + L7 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);




			gamma3sq = 1 / (L9 * L9 + L10 * L10 + L11 * L11);
			tmpA = gamma3sq * (L1 * L1 + L2 * L2 + L3 * L3) - x0 * x0;
			tmpB = gamma3sq * (L5 * L5 + L6 * L6 + L7 * L7) - y0 * y0;
			tmpC = gamma3sq * (L1 * L5 + L2 * L6 + L3 * L7) - x0 * y0;
			tmpNew = sqrt((tmpA * tmpB - tmpC * tmpC) / tmpB);

			//输出
			outfile << "---- 第" << i + 1 << "次迭代 ----" << std::endl;
			outfile << "L系数" << std::endl;
			outfile << L1 << " " << L2 << " " << L3 << " " << L4 << " " << std::endl;
			outfile << L5 << " " << L6 << " " << L7 << " " << L8 << " " << std::endl;
			outfile << L9 << " " << L10 << " " << L11 << " " << std::endl;
			outfile << "畸变参数" << std::endl;
			outfile << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " " << std::endl;
			outfile << "像主点坐标(pix)" << std::endl;
			outfile << "x0 " << x0 << " y0 " << y0 << " " << std::endl;
			outfile << "dfx(pix) " << tmpNew - tmpOld << std::endl;

			if (fabs(tmpNew - tmpOld) < 1e-4)
			{
				break;
			}
			tmpOld = tmpNew;
		}
		if (i < ITER_TIMES)
		{
			outfile << "======== 迭代成功！" << "共迭代" << i + 1 << "次 ======== " << std::endl;
			outfile << "参与平差的控制点个数:" << count << std::endl;
			Eigen::MatrixXd matV = matA * matX - matL;
			outfile << "像点残差(pix):" << std::endl;
			int j = 0;
			for (auto it = this->leftImgCtrCoor.begin(); it != leftImgCtrCoor.end(); it++, j++)
			{
				outfile << it->first << " vx: " << matV(j * 2, 0) << " vy: " << matV(j * 2 + 1, 0) << std::endl;
			}
			outfile << "======== 像点最或是值(pix) ========" << std::endl;
			j = 0;
			for (auto it = this->leftImgCtrCoor.begin(); it != leftImgCtrCoor.end(); it++, j++)
			{
				outfile << it->first << " " << matV(j * 2, 0) + it->second.x() << " " << matV(j * 2 + 1, 0) + it->second.y() << std::endl;
			}

			outfile << "======== 精度统计 ========" << std::endl;
			// 单位权中误差
			int r = num * 2 - 15;
			double sigma = sqrt((matV.transpose() * matV)(0, 0) / r);
			outfile << "单位权中误差：" << sigma << std::endl;
			//协因数阵
			Eigen::MatrixXd matQxhxh = (matA.transpose() * matA).inverse();
			//未知数中误差
			Eigen::MatrixXd matMxhxh = matQxhxh.diagonal().cwiseSqrt() * sigma;
			//std::cout << "diag(Qxhxh)" << std::endl << matQxhxh.diagonal() << std::endl;
			outfile << "未知数中误差：" << std::endl;
			outfile << "L系数" << std::endl;
			outfile << matMxhxh(0, 0) << " " << matMxhxh(1, 0) << " " << matMxhxh(2, 0) << " " << matMxhxh(3, 0) << " " << std::endl;
			outfile << matMxhxh(4, 0) << " " << matMxhxh(5, 0) << " " << matMxhxh(6, 0) << " " << matMxhxh(7, 0) << " " << std::endl;
			outfile << matMxhxh(8, 0) << " " << matMxhxh(9, 0) << " " << matMxhxh(10, 0) << " " << std::endl;
			outfile << "畸变参数" << std::endl;
			outfile << "k1 " << matMxhxh(11, 0) << " k2 " << matMxhxh(12, 0) << " p1 " << matMxhxh(13, 0) << " p2 " << matMxhxh(14, 0) << " " << std::endl;


			this->leftMatL(0, 0) = L1, this->leftMatL(0, 1) = L2, this->leftMatL(0, 2) = L3, this->leftMatL(0, 3) = L4;
			this->leftMatL(1, 0) = L5, this->leftMatL(1, 1) = L6, this->leftMatL(1, 2) = L7, this->leftMatL(1, 3) = L8;
			this->leftMatL(2, 0) = L9, this->leftMatL(2, 1) = L10, this->leftMatL(2, 2) = L11;
			this->left_x0 = x0, this->left_y0 = y0;

			this->leftDistortParam[0] = k1, this->leftDistortParam[1] = k2;
			this->leftDistortParam[0] = p1, this->leftDistortParam[1] = p2;

			outfile << "======== 计算结果 ========" << std::endl;
			outfile << "L系数" << std::endl;
			outfile << L1 << " " << L2 << " " << L3 << " " << L4 << " " << std::endl;
			outfile << L5 << " " << L6 << " " << L7 << " " << L8 << " " << std::endl;
			outfile << L9 << " " << L10 << " " << L11 << " " << std::endl;
			outfile << "畸变参数" << std::endl;
			outfile << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " " << std::endl;
			outfile << "像主点坐标(pix)" << std::endl;
			outfile << "x0 " << x0 << " y0 " << y0 << " " << std::endl;

			{
				outfile << "外方位线元素(mm)" << std::endl;
				Eigen::MatrixXd tmpXYZ(3, 1);
				Eigen::Vector3d tmpL(3, 1);
				tmpL(0, 0) = -L4; tmpL(1, 0) = -L8; tmpL(2, 0) = -1;
				Eigen::Matrix3d tmpA;
				tmpA(0, 0) = L1; tmpA(0, 1) = L2; tmpA(0, 2) = L3;
				tmpA(1, 0) = L5; tmpA(1, 1) = L6; tmpA(1, 2) = L7;
				tmpA(2, 0) = L9; tmpA(2, 1) = L10; tmpA(2, 2) = L11;
				tmpXYZ = tmpA.inverse() * tmpL;
				outfile << "Xs " << tmpXYZ(0, 0) << " Ys " << tmpXYZ(1, 0) << " Zs " << tmpXYZ(2, 0) << std::endl;


				double gamma3sq = 1 / (L9 * L9 + L10 * L10 + L11 * L11);
				double A = gamma3sq * (L1 * L1 + L2 * L2 + L3 * L3) - x0 * x0;
				double B = gamma3sq * (L5 * L5 + L6 * L6 + L7 * L7) - y0 * y0;
				double C = gamma3sq * (L1 * L5 + L2 * L6 + L3 * L7) - x0 * y0;
				double fx = sqrt((A * B - C * C) / B);
				outfile << "主距fx(pix) " << fx << std::endl;

				double dbeta = C < 0 ? asin(sqrt(C * C / (A * B))) : asin(-sqrt(C * C / (A * B)));
				double ds = sqrt(A / B) - 1;
				outfile << "比例尺不一系数ds " << ds << " 不正交性角dbeta " << dbeta << std::endl;


				outfile << "外方位角元素(rad)" << std::endl;
				double a3 = sqrt(gamma3sq) * L9;
				double b3 = sqrt(gamma3sq) * L10;
				double c3 = sqrt(gamma3sq) * L11;
				double b2 = sqrt(gamma3sq) * (L6 + L10 * y0) * (1 + ds) * cos(dbeta) / fx;
				double b1 = (L2 * sqrt(gamma3sq) + b3 * x0 + b2 * fx * tan(dbeta)) / fx;

				outfile << "phi " << atan(-a3 / c3) << " omega " << atan(-b3) << " kappa " << atan(b1 / b2) << std::endl;

			}
		}
		outfile.close();
	}

	//---- 右像片“后方交会” ----
	{
		outfile.open("output/" + this->mstr + "_RightL.rep", std::ios::trunc);
		if (!outfile.is_open())
		{
			std::cout << "output failed" << std::endl;
			return;
		}

		//控制点个数
		int num = this->rightImgCtrCoor.size();
		//计数变量
		int count = 0, i = 0;
		//内方位元素
		double x0 = this->right_x0, y0 = this->right_y0;
		//物方点的物方坐标
		double X, Y, Z;
		//像点坐标
		double x, y;
		//L系数
		double L1, L2, L3, L4;
		double L5, L6, L7, L8;
		double L9, L10, L11;
		//分母A
		double A = 1;
		// r = 径向
		double r;
		//畸变系数
		double k1 = this->rightDistortParam[0], k2 = this->rightDistortParam[1];
		double p1 = this->rightDistortParam[0], p2 = this->rightDistortParam[1];
		//法方程系数阵
		Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(2 * num, 15);
		Eigen::MatrixXd matX = Eigen::MatrixXd::Zero(15, 1);	//L1 ... L11 k1 k2 p1 p2
		//法方程常数项
		Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(2 * num, 1);
		//记录上一次迭代的fx的数值(pix)
		double tmpOld, tmpNew;
		double tmpA;
		double tmpB;
		double tmpC;
		double gamma3sq;

		x0 = this->right_x0, y0 = this->right_y0;
		L1 = this->rightMatL(0, 0), L2 = this->rightMatL(0, 1), L3 = this->rightMatL(0, 2), L4 = this->rightMatL(0, 3);
		L5 = this->rightMatL(1, 0), L6 = this->rightMatL(1, 1), L7 = this->rightMatL(1, 2), L8 = this->rightMatL(1, 3);
		L9 = this->rightMatL(2, 0), L10 = this->rightMatL(2, 1), L11 = this->rightMatL(2, 2);

		outfile << "======== 初值计算结果 ========" << std::endl;
		outfile << "L系数" << std::endl;
		outfile << L1 << " " << L2 << " " << L3 << " " << L4 << " " << std::endl;
		outfile << L5 << " " << L6 << " " << L7 << " " << L8 << " " << std::endl;
		outfile << L9 << " " << L10 << " " << L11 << " " << std::endl;
		outfile << "畸变参数" << std::endl;
		outfile << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " " << std::endl;
		outfile << "像主点坐标(pix)" << std::endl;
		outfile << "x0 " << x0 << " y0 " << y0 << " " << std::endl;

		gamma3sq = 1 / (L9 * L9 + L10 * L10 + L11 * L11);
		tmpA = gamma3sq * (L1 * L1 + L2 * L2 + L3 * L3) - x0 * x0;
		tmpB = gamma3sq * (L5 * L5 + L6 * L6 + L7 * L7) - y0 * y0;
		tmpC = gamma3sq * (L1 * L5 + L2 * L6 + L3 * L7) - x0 * y0;
		tmpOld = sqrt((tmpA * tmpB - tmpC * tmpC) / tmpB);

		outfile << "======== 迭代计算结果 ========" << std::endl;
		for (i = 0; i < ITER_TIMES; i++)
		{
			count = 0;
			//对每个像点列方程
			for (auto it = this->rightImgCtrCoor.begin(); it != rightImgCtrCoor.end(); it++)
			{
				x = it->second.x();
				y = it->second.y();
				// 对每一个像点搜索它的物方坐标
				X = this->ctrObjCoor.at(it->first).x();
				Y = this->ctrObjCoor.at(it->first).y();
				Z = this->ctrObjCoor.at(it->first).z();

				r = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
				A = L9 * X + L10 * Y + L11 * Z + 1;


				matL(2 * count, 0) = x / A;
				matL(2 * count + 1, 0) = y / A;

				matA(2 * count, 0) = X / A;
				matA(2 * count, 1) = Y / A;
				matA(2 * count, 2) = Z / A;
				matA(2 * count, 3) = 1 / A;
				matA(2 * count, 4) = 0;
				matA(2 * count, 5) = 0;
				matA(2 * count, 6) = 0;
				matA(2 * count, 7) = 0;
				matA(2 * count, 8) = x * X / A;
				matA(2 * count, 9) = x * Y / A;
				matA(2 * count, 10) = x * Z / A;
				matA(2 * count, 11) = (x - x0) * pow(r, 2);
				matA(2 * count, 12) = (x - x0) * pow(r, 4);
				matA(2 * count, 13) = pow(r, 2) + 2 * pow(x - x0, 2);
				matA(2 * count, 14) = 2 * (x - x0) * (y - y0);

				matA(2 * count + 1, 0) = 0;
				matA(2 * count + 1, 1) = 0;
				matA(2 * count + 1, 2) = 0;
				matA(2 * count + 1, 3) = 0;
				matA(2 * count + 1, 4) = X / A;
				matA(2 * count + 1, 5) = Y / A;
				matA(2 * count + 1, 6) = Z / A;
				matA(2 * count + 1, 7) = 1 / A;
				matA(2 * count + 1, 8) = y * X / A;
				matA(2 * count + 1, 9) = y * Y / A;
				matA(2 * count + 1, 10) = y * Z / A;
				matA(2 * count + 1, 11) = (y - y0) * pow(r, 2);
				matA(2 * count + 1, 12) = (y - y0) * pow(r, 4);
				matA(2 * count + 1, 13) = 2 * (x - x0) * (y - y0);
				matA(2 * count + 1, 14) = pow(r, 2) + 2 * pow(y - y0, 2);

				count++;
			}
			matA = -matA;
			//计算新值
			matX = (matA.transpose() * matA).inverse() * matA.transpose() * matL;

			//更新未知数 L1 ... L11 k1 k2 p1 p2 
			L1 = matX(0, 0);
			L2 = matX(1, 0);
			L3 = matX(2, 0);
			L4 = matX(3, 0);
			L5 = matX(4, 0);
			L6 = matX(5, 0);
			L7 = matX(6, 0);
			L8 = matX(7, 0);
			L9 = matX(8, 0);
			L10 = matX(9, 0);
			L11 = matX(10, 0);
			k1 = matX(11, 0);
			k2 = matX(12, 0);
			p1 = matX(13, 0);
			p2 = matX(14, 0);
			//更新内方位元素
			x0 = -(L1 * L9 + L2 * L10 + L3 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);
			y0 = -(L5 * L9 + L6 * L10 + L7 * L11) / (L9 * L9 + L10 * L10 + L11 * L11);

			gamma3sq = 1 / (L9 * L9 + L10 * L10 + L11 * L11);
			tmpA = gamma3sq * (L1 * L1 + L2 * L2 + L3 * L3) - x0 * x0;
			tmpB = gamma3sq * (L5 * L5 + L6 * L6 + L7 * L7) - y0 * y0;
			tmpC = gamma3sq * (L1 * L5 + L2 * L6 + L3 * L7) - x0 * y0;
			tmpNew = sqrt((tmpA * tmpB - tmpC * tmpC) / tmpB);

			//输出
			outfile << "---- 第" << i + 1 << "次迭代 ----" << std::endl;
			outfile << "L系数" << std::endl;
			outfile << L1 << " " << L2 << " " << L3 << " " << L4 << " " << std::endl;
			outfile << L5 << " " << L6 << " " << L7 << " " << L8 << " " << std::endl;
			outfile << L9 << " " << L10 << " " << L11 << " " << std::endl;
			outfile << "畸变参数" << std::endl;
			outfile << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " " << std::endl;
			outfile << "像主点坐标(pix)" << std::endl;
			outfile << "x0 " << x0 << " y0 " << y0 << " " << std::endl;
			outfile << "dfx(pix) " << tmpNew - tmpOld << std::endl;


			if (fabs(tmpNew - tmpOld) < 1e-4)
			{
				break;
			}
			tmpOld = tmpNew;
		}
		if (i < ITER_TIMES)
		{
			outfile << "======== 迭代成功！" << "共迭代" << i + 1 << "次 ======== " << std::endl;
			outfile << "参与平差的控制点个数:" << count << std::endl;
			Eigen::MatrixXd matV = matA * matX - matL;
			outfile << "像点残差(pix):" << std::endl;
			int j = 0;
			for (auto it = this->rightImgCtrCoor.begin(); it != rightImgCtrCoor.end(); it++, j++)
			{
				outfile << it->first << " vx: " << matV(j * 2, 0) << " vy: " << matV(j * 2 + 1, 0) << std::endl;
			}
			outfile << "======== 像点最或是值(pix) ========" << std::endl;
			j = 0;
			for (auto it = this->rightImgCtrCoor.begin(); it != rightImgCtrCoor.end(); it++, j++)
			{
				outfile << it->first << " " << matV(j * 2, 0) + it->second.x() << " " << matV(j * 2 + 1, 0) + it->second.y() << std::endl;
			}

			outfile << "======== 精度统计 ========" << std::endl;
			// 单位权中误差
			int r = num * 2 - 15;
			double sigma = sqrt((matV.transpose() * matV)(0, 0) / r);
			outfile << "单位权中误差：" << sigma << std::endl;
			//协因数阵
			Eigen::MatrixXd matQxhxh = (matA.transpose() * matA).inverse();
			//未知数中误差
			Eigen::MatrixXd matMxhxh = matQxhxh.diagonal().cwiseSqrt() * sigma;
			//std::cout << "diag(Qxhxh)" << std::endl << matQxhxh.diagonal() << std::endl;
			outfile << "未知数中误差：" << std::endl;
			outfile << "L系数" << std::endl;
			outfile << matMxhxh(0, 0) << " " << matMxhxh(1, 0) << " " << matMxhxh(2, 0) << " " << matMxhxh(3, 0) << " " << std::endl;
			outfile << matMxhxh(4, 0) << " " << matMxhxh(5, 0) << " " << matMxhxh(6, 0) << " " << matMxhxh(7, 0) << " " << std::endl;
			outfile << matMxhxh(8, 0) << " " << matMxhxh(9, 0) << " " << matMxhxh(10, 0) << " " << std::endl;
			outfile << "畸变参数" << std::endl;
			outfile << "k1 " << matMxhxh(11, 0) << " k2 " << matMxhxh(12, 0) << " p1 " << matMxhxh(13, 0) << " p2 " << matMxhxh(14, 0) << " " << std::endl;



			this->rightMatL(0, 0) = L1, this->rightMatL(0, 1) = L2, this->rightMatL(0, 2) = L3, this->rightMatL(0, 3) = L4;
			this->rightMatL(1, 0) = L5, this->rightMatL(1, 1) = L6, this->rightMatL(1, 2) = L7, this->rightMatL(1, 3) = L8;
			this->rightMatL(2, 0) = L9, this->rightMatL(2, 1) = L10, this->rightMatL(2, 2) = L11;
			this->right_x0 = x0, this->right_y0 = y0;

			this->rightDistortParam[0] = k1, this->rightDistortParam[1] = k2;
			this->rightDistortParam[0] = p1, this->rightDistortParam[1] = p2;

			outfile << "======== 计算结果 ========" << std::endl;
			outfile << "L系数" << std::endl;
			outfile << L1 << " " << L2 << " " << L3 << " " << L4 << " " << std::endl;
			outfile << L5 << " " << L6 << " " << L7 << " " << L8 << " " << std::endl;
			outfile << L9 << " " << L10 << " " << L11 << " " << std::endl;
			outfile << "畸变参数" << std::endl;
			outfile << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " " << std::endl;
			outfile << "像主点坐标(pix)" << std::endl;
			outfile << "x0 " << x0 << " y0 " << y0 << " " << std::endl;

			{
				outfile << "外方位线元素(mm)" << std::endl;
				Eigen::MatrixXd tmpXYZ(3, 1);
				Eigen::Vector3d tmpL(3, 1);
				tmpL(0, 0) = -L4; tmpL(1, 0) = -L8; tmpL(2, 0) = -1;
				Eigen::Matrix3d tmpA;
				tmpA(0, 0) = L1; tmpA(0, 1) = L2; tmpA(0, 2) = L3;
				tmpA(1, 0) = L5; tmpA(1, 1) = L6; tmpA(1, 2) = L7;
				tmpA(2, 0) = L9; tmpA(2, 1) = L10; tmpA(2, 2) = L11;
				tmpXYZ = tmpA.inverse() * tmpL;
				outfile << "Xs " << tmpXYZ(0, 0) << " Ys " << tmpXYZ(1, 0) << " Zs " << tmpXYZ(2, 0) << std::endl;


				double gamma3sq = 1 / (L9 * L9 + L10 * L10 + L11 * L11);
				double A = gamma3sq * (L1 * L1 + L2 * L2 + L3 * L3) - x0 * x0;
				double B = gamma3sq * (L5 * L5 + L6 * L6 + L7 * L7) - y0 * y0;
				double C = gamma3sq * (L1 * L5 + L2 * L6 + L3 * L7) - x0 * y0;
				double fx = sqrt((A * B - C * C) / B);
				outfile << "主距fx(pix) " << fx << std::endl;

				double dbeta = C < 0 ? asin(sqrt(C * C / (A * B))) : asin(-sqrt(C * C / (A * B)));
				double ds = sqrt(A / B) - 1;
				outfile << "比例尺不一系数ds " << ds << " 不正交性角dbeta " << dbeta << std::endl;


				outfile << "外方位角元素(rad)" << std::endl;
				double a3 = sqrt(gamma3sq) * L9;
				double b3 = sqrt(gamma3sq) * L10;
				double c3 = sqrt(gamma3sq) * L11;
				double b2 = sqrt(gamma3sq) * (L6 + L10 * y0) * (1 + ds) * cos(dbeta) / fx;
				double b1 = (L2 * sqrt(gamma3sq) + b3 * x0 + b2 * fx * tan(dbeta)) / fx;

				outfile << "phi " << atan(-a3 / c3) << " omega " << atan(-b3) << " kappa " << atan(b1 / b2) << std::endl;

			}


		}
		outfile.close();
	}

}

void DLT::correctImgCoor()
{
	//合并待定点与检查点的map
	this->leftImgUknCoor.insert(this->leftImgUknCoor.begin(), this->leftImgUknCoor.end());
	this->rightImgUknCoor.insert(this->rightImgUknCoor.begin(), this->rightImgUknCoor.end());

	std::map<int, Eigen::Vector2d> tmpLeft;
	std::map<int, Eigen::Vector2d> tmpRight;

	//先改正
	double x, y;
	double x0, y0;
	double r, dx, dy;
	double k1, k2;
	double p1, p2;
	for (auto it = this->leftImgUknCoor.begin(); it != this->leftImgUknCoor.end(); it++)
	{
		x = it->second.x();
		y = it->second.y();
		x0 = this->left_x0;
		y0 = this->left_y0;

		r = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
		k1 = this->leftDistortParam[0], k2 = this->leftDistortParam[1];
		p1 = this->leftDistortParam[0], p2 = this->leftDistortParam[1];

		dx = k1 * (x - x0) * pow(r, 2) + k2 * (x - x0) * pow(r, 4) + p1 * (pow(r, 2) + 2 * pow(x - x0, 2)) + 2 * p2 * (x - x0) * (y - y0);
		dy = k1 * (y - y0) * pow(r, 2) + k2 * (y - y0) * pow(r, 4) + p2 * (pow(r, 2) + 2 * pow(y - y0, 2)) + 2 * p1 * (x - x0) * (y - y0);

		tmpLeft[it->first] = Eigen::Vector2d(x + dx, y + dy);
	}
	for (auto it = this->rightImgUknCoor.begin(); it != this->rightImgUknCoor.end(); it++)
	{
		x = it->second.x();
		y = it->second.y();
		x0 = this->right_x0;
		y0 = this->right_y0;

		r = sqrt(pow(x - x0, 2) + pow(y - y0, 2));
		k1 = this->rightDistortParam[0], k2 = this->rightDistortParam[1];
		p1 = this->rightDistortParam[0], p2 = this->rightDistortParam[1];

		dx = k1 * (x - x0) * pow(r, 2) + k2 * (x - x0) * pow(r, 4) + p1 * (pow(r, 2) + 2 * pow(x - x0, 2)) + 2 * p2 * (x - x0) * (y - y0);
		dy = k1 * (y - y0) * pow(r, 2) + k2 * (y - y0) * pow(r, 4) + p2 * (pow(r, 2) + 2 * pow(y - y0, 2)) + 2 * p1 * (x - x0) * (y - y0);

		tmpRight[it->first] = Eigen::Vector2d(x + dx, y + dy);
	}

	//再配对
	this->leftImgCorCoor.clear();
	this->rightImgCorCoor.clear();


	auto it1 = tmpLeft.begin();
	auto it2 = tmpRight.begin();
	while (it1 != tmpLeft.end() && it2 != tmpRight.end())
	{
		if (it1->first < it2->first)
			it1++;
		else if (it1->first > it2->first)
			it2++;
		else
		{
			this->leftImgCorCoor[it1->first] = it1->second;
			this->rightImgCorCoor[it2->first] = it2->second;
			it1++;
			it2++;
		}
	}

}

void DLT::initUknObjCoor()
{
	this->uknObjCoor.clear();

	Eigen::MatrixXd matA();
	int ID;
	double Left_x, Left_y;
	double Right_x, Right_y;

	double LeftL1, LeftL2, LeftL3, LeftL4;
	double LeftL5, LeftL6, LeftL7, LeftL8;
	double LeftL9, LeftL10, LeftL11;

	double RightL1, RightL2, RightL3, RightL4;
	double RightL5, RightL6, RightL7, RightL8;
	double RightL9, RightL10, RightL11;

	LeftL1 = this->leftMatL(0, 0), LeftL2 = this->leftMatL(0, 1), LeftL3 = this->leftMatL(0, 2), LeftL4 = this->leftMatL(0, 3);
	LeftL5 = this->leftMatL(1, 0), LeftL6 = this->leftMatL(1, 1), LeftL7 = this->leftMatL(1, 2), LeftL8 = this->leftMatL(1, 3);
	LeftL9 = this->leftMatL(2, 0), LeftL10 = this->leftMatL(2, 1), LeftL11 = this->leftMatL(2, 2);

	RightL1 = this->rightMatL(0, 0), RightL2 = this->rightMatL(0, 1), RightL3 = this->rightMatL(0, 2), RightL4 = this->rightMatL(0, 3);
	RightL5 = this->rightMatL(1, 0), RightL6 = this->rightMatL(1, 1), RightL7 = this->rightMatL(1, 2), RightL8 = this->rightMatL(1, 3);
	RightL9 = this->rightMatL(2, 0), RightL10 = this->rightMatL(2, 1), RightL11 = this->rightMatL(2, 2);


	//待定点物方坐标近似值计算
	{
		Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(4, 3);
		Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(4, 1);

		for (auto it = this->leftImgCorCoor.begin(); it != this->leftImgCorCoor.end(); it++)
		{
			ID = it->first;

			//左片
			Left_x = it->second.x();
			Left_y = it->second.y();

			matA(0, 0) = LeftL1 + Left_x * LeftL9;
			matA(0, 1) = LeftL2 + Left_x * LeftL10;
			matA(0, 2) = LeftL3 + Left_x * LeftL11;
			matL(0, 0) = -(LeftL4 + Left_x);

			matA(1, 0) = LeftL5 + Left_y * LeftL9;
			matA(1, 1) = LeftL6 + Left_y * LeftL10;
			matA(1, 2) = LeftL7 + Left_y * LeftL11;
			matL(1, 0) = -(LeftL8 + Left_y);

			//右片
			Right_x = this->rightImgCorCoor.at(ID).x();
			Right_y = this->rightImgCorCoor.at(ID).y();

			matA(2, 0) = RightL1 + Right_x * RightL9;
			matA(2, 1) = RightL2 + Right_x * RightL10;
			matA(2, 2) = RightL3 + Right_x * RightL11;
			matL(2, 0) = -(RightL4 + Right_x);

			matA(3, 0) = RightL5 + Right_y * RightL9;
			matA(3, 1) = RightL6 + Right_y * RightL10;
			matA(3, 2) = RightL7 + Right_y * RightL11;
			matL(3, 0) = -(RightL8 + Right_y);

			Eigen::MatrixXd temp = (matA.transpose() * matA).inverse() * matA.transpose() * matL;
			this->uknObjCoor[ID] = Eigen::Vector3d(temp(0, 0), temp(1, 0), temp(2, 0));
		}
	}

}

void DLT::calculateUknObjCoor()
{
	//改正待定点的像点坐标
	correctImgCoor();
	//计算待定点的物方坐标初始值
	initUknObjCoor();

	std::ofstream outfile;   //输出流
	std::map<int, double> tmpM; //记录单位权中误差
	std::map<int, Eigen::MatrixXd> tmpMx; //记录未知数中误差

	outfile.open("output/" + this->mstr + "_uknCoor.rep", std::ios::trunc);
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
		return;
	}

	//计算待定点的物方坐标精确值
	{
		int ID;
		int i = 0; //计数变量
		double X = 0, Y = 0, Z = 0;
		double Left_x, Left_y;
		double Right_x, Right_y;
		double LeftA, RightA;

		double LeftL1, LeftL2, LeftL3, LeftL4;
		double LeftL5, LeftL6, LeftL7, LeftL8;
		double LeftL9, LeftL10, LeftL11;

		double RightL1, RightL2, RightL3, RightL4;
		double RightL5, RightL6, RightL7, RightL8;
		double RightL9, RightL10, RightL11;

		LeftL1 = this->leftMatL(0, 0), LeftL2 = this->leftMatL(0, 1), LeftL3 = this->leftMatL(0, 2), LeftL4 = this->leftMatL(0, 3);
		LeftL5 = this->leftMatL(1, 0), LeftL6 = this->leftMatL(1, 1), LeftL7 = this->leftMatL(1, 2), LeftL8 = this->leftMatL(1, 3);
		LeftL9 = this->leftMatL(2, 0), LeftL10 = this->leftMatL(2, 1), LeftL11 = this->leftMatL(2, 2);

		RightL1 = this->rightMatL(0, 0), RightL2 = this->rightMatL(0, 1), RightL3 = this->rightMatL(0, 2), RightL4 = this->rightMatL(0, 3);
		RightL5 = this->rightMatL(1, 0), RightL6 = this->rightMatL(1, 1), RightL7 = this->rightMatL(1, 2), RightL8 = this->rightMatL(1, 3);
		RightL9 = this->rightMatL(2, 0), RightL10 = this->rightMatL(2, 1), RightL11 = this->rightMatL(2, 2);


		Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(4, 3);
		Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(4, 1);
		Eigen::MatrixXd matX;
		double tmpOld, tmpNew;

		outfile << "======== 逐点迭代计算结果 ========" << std::endl;
		outfile << "共解算" << this->leftImgCorCoor.size() << "个物方点，其中待定点" << this->leftImgCorCoor.size() - this->leftImgUknCoor.size() << "个"
			<< "，检查点" << this->leftImgUknCoor.size() << "个" << std::endl;
		for (auto it = this->leftImgCorCoor.begin(); it != this->leftImgCorCoor.end(); it++)
		{
			ID = it->first;

			Left_x = it->second.x();
			Left_y = it->second.y();

			Right_x = this->rightImgCorCoor.at(ID).x();
			Right_y = this->rightImgCorCoor.at(ID).y();

			tmpOld = this->uknObjCoor.at(ID).x();


			outfile << "==== " << ID << " ====" << std::endl;
			outfile << "初值X,Y,Z(mm)" << std::endl;
			outfile << this->uknObjCoor.at(ID).x() << " " << this->uknObjCoor.at(ID).y() << " " << this->uknObjCoor.at(ID).z() << " " << std::endl;

			for (i = 0; i < ITER_TIMES; i++)
			{
				X = this->uknObjCoor.at(ID).x();
				Y = this->uknObjCoor.at(ID).y();
				Z = this->uknObjCoor.at(ID).z();

				LeftA = LeftL9 * X + LeftL10 * Y + LeftL11 * Z + 1;
				RightA = RightL9 * X + RightL10 * Y + RightL11 * Z + 1;

				//左片
				matA(0, 0) = -(LeftL1 + Left_x * LeftL9) / LeftA;
				matA(0, 1) = -(LeftL2 + Left_x * LeftL10) / LeftA;
				matA(0, 2) = -(LeftL3 + Left_x * LeftL11) / LeftA;
				matL(0, 0) = (LeftL4 + Left_x) / LeftA;

				matA(1, 0) = -(LeftL5 + Left_y * LeftL9) / LeftA;
				matA(1, 1) = -(LeftL6 + Left_y * LeftL10) / LeftA;
				matA(1, 2) = -(LeftL7 + Left_y * LeftL11) / LeftA;
				matL(1, 0) = (LeftL8 + Left_y) / LeftA;

				//右片
				matA(2, 0) = -(RightL1 + Right_x * RightL9) / RightA;
				matA(2, 1) = -(RightL2 + Right_x * RightL10) / RightA;
				matA(2, 2) = -(RightL3 + Right_x * RightL11) / RightA;
				matL(2, 0) = (RightL4 + Right_x) / RightA;

				matA(3, 0) = -(RightL5 + Right_y * RightL9) / RightA;
				matA(3, 1) = -(RightL6 + Right_y * RightL10) / RightA;
				matA(3, 2) = -(RightL7 + Right_y * RightL11) / RightA;
				matL(3, 0) = (RightL8 + Right_y) / RightA;

				matX = (matA.transpose() * matA).inverse() * matA.transpose() * matL;
				tmpNew = matX(0, 0);

				outfile << "---- 第" << i + 1 << "次迭代 ----" << std::endl;
				outfile << "物方坐标X,Y,Z(mm)" << std::endl;
				outfile << matX(0, 0) << " " << matX(1, 0) << " " << matX(2, 0) << std::endl;

				if (fabs(tmpNew - tmpOld) < 1e-10)
				{
					break;
				}
				tmpOld = tmpNew;

				X = matX(0, 0);
				Y = matX(1, 0);
				Z = matX(2, 0);

			}
			if (i < ITER_TIMES)
			{
				this->uknObjCoor[ID] = Eigen::Vector3d(matX(0, 0), matX(1, 0), matX(2, 0));
				outfile << "---- " << "迭代成功！" << "共迭代" << i + 1 << "次 ----" << std::endl;

				Eigen::MatrixXd matV = matA * matX - matL;

				outfile << "物方坐标X,Y,Z(mm)" << std::endl;
				outfile << matX(0, 0) << " " << matX(1, 0) << " " << matX(2, 0) << std::endl;

				outfile << "左片像点残差vx,vy(pix)" << std::endl;
				outfile << matV(0, 0) << " " << matV(1, 0) << std::endl;
				outfile << "右片像点残差vx,vy(pix)" << std::endl;
				outfile << matV(2, 0) << " " << matV(3, 0) << std::endl;

				// 单位权中误差
				int r = 2 * 2 - 3;
				double sigma = sqrt((matV.transpose() * matV)(0, 0) / r);
				tmpM[ID] = sigma;
				outfile << "单位权中误差：" << sigma << std::endl;

				//协因数阵
				Eigen::MatrixXd matQxhxh = (matA.transpose() * matA).inverse();
				//未知数中误差
				Eigen::MatrixXd matMxhxh = matQxhxh.diagonal().cwiseSqrt() * sigma;
				tmpMx[ID] = matMxhxh;

			}
		}
	}

	//输出物方坐标
	{
		int ID = 0;
		outfile << "======== 计算结果(mm) ========" << std::endl;
		outfile << "X Y Z m0 mX mY mZ [flag(is checking points)]" << std::endl;
		for (auto it = this->uknObjCoor.begin(); it != this->uknObjCoor.end(); it++)
		{
			ID = it->first;
			auto it1 = this->chkObjCoor.find(ID);
			outfile << ID << " " << it->second.x() << " " << it->second.y() << " " << it->second.z() << " " << tmpM.at(ID) << " "
				<< tmpMx.at(ID)(0, 0) << " " << tmpMx.at(ID)(1, 0) << " " << tmpMx.at(ID)(2, 0) << " " << (it1 != this->chkObjCoor.end()) << std::endl;
		}
	}

	//计算外精度
	{
		int num = 0;
		int ID;
		double sumdX2 = 0, sumdY2 = 0, sumdZ2 = 0;
		outfile << "======== 检查点计算外精度(mm) ========" << std::endl;
		outfile << "ID vX vY vZ" << std::endl;
		for (auto it = this->chkObjCoor.begin(); it != this->chkObjCoor.end(); it++)
		{
			ID = it->first;
			auto it1 = this->uknObjCoor.find(ID);

			if (it1 != this->uknObjCoor.end())
			{
				outfile << ID << " " << it->second.x() - it1->second.x() << " " << it->second.y() - it1->second.y() << " " << it->second.z() - it1->second.z() << std::endl;
				sumdX2 += pow(it->second.x() - it1->second.x(), 2);
				sumdY2 += pow(it->second.y() - it1->second.y(), 2);
				sumdZ2 += pow(it->second.z() - it1->second.z(), 2);
				num++;
				this->uknObjCoor.erase(it1);

				auto it2 = this->leftImgUknCoor.find(ID);
				this->leftImgUknCoor.erase(it2);
				auto it3 = this->rightImgUknCoor.find(ID);
				this->rightImgUknCoor.erase(it3);
				auto it4 = this->leftImgCorCoor.find(ID);
				this->leftImgCorCoor.erase(it4);
				auto it5 = this->rightImgCorCoor.find(ID);
				this->rightImgCorCoor.erase(it5);
			}
		}
		outfile << "X坐标中误差: " << sqrt(sumdX2 / num) << std::endl;
		outfile << "Y坐标中误差: " << sqrt(sumdY2 / num) << std::endl;
		outfile << "Z坐标中误差: " << sqrt(sumdZ2 / num) << std::endl;
		outfile << "点位中误差： " << sqrt((sumdX2 + sumdY2 + sumdZ2) / num) << std::endl;


	}
	outfile.close();

}
