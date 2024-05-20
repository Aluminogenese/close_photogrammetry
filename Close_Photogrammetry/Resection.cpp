#include "Resection.h"

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
	BaseClass::getImageCoor(imgPath, this->imageCoor);
	std::map<int, Eigen::Vector3d> originObjCoor;
	BaseClass::getObjCoor(objPath, originObjCoor);
	BaseClass::leftHand2RightHand(originObjCoor, this->objCoor);
}

void Resection::calculate(const std::string& filePath)
{
	//���Ƶ����
	int num = this->imageCoor.size();
	//��������
	int count = 0, i = 0;
	//�ⷽλ��Ԫ��
	double Xs = this->ext_elements[0], Ys = this->ext_elements[1], Zs = this->ext_elements[2];
	//�ⷽλ��Ԫ��
	double phi = this->ext_elements[3], omega = this->ext_elements[4], kappa = this->ext_elements[5];
	//�ڷ�λԪ��
	double f = this->int_elements[0], x0 = this->int_elements[1], y0 = this->int_elements[2];
	//�﷽����﷽����
	double X, Y, Z;
	//�﷽������ռ�����ϵ�е�����
	double Xbar, Ybar, Zbar;
	//�������
	double x, y;
	//�񷽵ĸ�������
	double dx, dy;
	// r = ����
	double r;
	//����ϵ��
	double k1 = this->distort_param[0], k2 = this->distort_param[1];
	double p1 = this->distort_param[2], p2 = this->distort_param[3];


	//��ת����Ԫ��
	double a1, a2, a3;
	double b1, b2, b3;
	double c1, c2, c3;

	// ������ϵ����
	Eigen::MatrixXd matA = Eigen::MatrixXd::Zero(2 * num, 13);
	Eigen::MatrixXd matX = Eigen::MatrixXd::Zero(13, 1);	// Xs Ys Zs phi omega kappa f x0 y0 k1 k2 p1 p2
	Eigen::MatrixXd matL = Eigen::MatrixXd::Zero(2 * num, 1);

	//������
	std::ofstream outfile(filePath, std::ios::trunc);  //�����
	if (!outfile.is_open())
	{
		std::cout << "output failed" << std::endl;
		return;
	}
	outfile << "======== ���������� ========" << std::endl;
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
		//��ÿ������з���
		for (auto it = this->imageCoor.begin(); it != imageCoor.end(); it++)
		{
			x = it->second.x();
			y = it->second.y();
			// ��ÿһ��������������﷽����
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

			//�ⷽλԪ��
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
			//�ڷ�λԪ��
			matA(2 * count, 6) = (x - x0) / f;
			matA(2 * count, 7) = 1;
			matA(2 * count, 8) = 0;
			//����ϵ��
			matA(2 * count, 9) = -(x - x0) * pow(r, 2);
			matA(2 * count, 10) = -(x - x0) * pow(r, 4);
			matA(2 * count, 11) = -(pow(r, 2) + 2 * pow(x - x0, 2));
			matA(2 * count, 12) = -2 * (x - x0) * (y - y0);

			//�ⷽλԪ��
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
			matA(2 * count + 1, 5) = -(x - x0);;
			//�ڷ�λԪ��
			matA(2 * count + 1, 6) = (y - y0) / f;
			matA(2 * count + 1, 7) = 0;
			matA(2 * count + 1, 8) = 1;
			//����ϵ��
			matA(2 * count + 1, 9) = -(y - y0) * pow(r, 2);
			matA(2 * count + 1, 10) = -(y - y0) * pow(r, 4);
			matA(2 * count + 1, 11) = -2 * (x - x0) * (y - y0);
			matA(2 * count + 1, 12) = -(pow(r, 2) + 2 * pow(y - y0, 2));

			count++;
		}
		// ���������
		matX = (matA.transpose() * matA).inverse() * matA.transpose() * matL;
		outfile << "---- ��" << i + 1 << "�ε��� ----" << std::endl;
		outfile << matX << std::endl;
		if (fabs(matX(3, 0)) < 1e-10)
		{
			break;
		}
		//����δ֪�� Xs Ys Zs phi omega kappa f x0 y0 k1 k2 p1 p2
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
		outfile << "======== �����ɹ���" << "������" << i + 1 << "�� ========" << std::endl;
		outfile << "����ƽ��Ŀ��Ƶ����:" << count << std::endl;
		Eigen::MatrixXd matV = matA * matX - matL;
		outfile << "���в�(pix):" << std::endl;
		int i = 0;
		for (auto it = this->imageCoor.begin(); it != imageCoor.end(); it++, i++)
		{
			outfile << it->first << " vx: " << matV(i * 2, 0) << " vy: " << matV(i * 2 + 1, 0) << std::endl;
		}
		outfile << "======== ����ͳ�� ========" << std::endl;
		// ��λȨ�����
		int r = num * 2 - 13;
		double sigma = sqrt((matV.transpose() * matV)(0, 0) / r);
		outfile << "��λȨ�����(pix)��" << sigma << std::endl;
		//Э������
		Eigen::MatrixXd matQxhxh = (matA.transpose() * matA).inverse();
		//δ֪�������
		Eigen::MatrixXd matMxhxh = matQxhxh.diagonal().cwiseSqrt() * sigma;
		//outfile << "diag(Qxhxh)" << std::endl << matQxhxh.diagonal() << std::endl;
		outfile << "δ֪������" << std::endl;
		outfile << "Xs(mm)��" << matMxhxh(0, 0) << std::endl;
		outfile << "Ys(mm)��" << matMxhxh(1, 0) << std::endl;
		outfile << "Zs(mm)��" << matMxhxh(2, 0) << std::endl;
		outfile << "phi(rad)��" << matMxhxh(3, 0) << std::endl;
		outfile << "omega(rad)��" << matMxhxh(4, 0) << std::endl;
		outfile << "kappa(rad)��" << matMxhxh(5, 0) << std::endl;
		outfile << "f(pix)��" << matMxhxh(6, 0) << std::endl;
		outfile << "x0(pix)��" << matMxhxh(7, 0) << std::endl;
		outfile << "y0(pix)��" << matMxhxh(8, 0) << std::endl;
		outfile << "k1(pix^-2)��" << matMxhxh(9, 0) << std::endl;
		outfile << "k2(pix^-4)��" << matMxhxh(10, 0) << std::endl;
		outfile << "p1(pix^-1)��" << matMxhxh(11, 0) << std::endl;
		outfile << "p2(pix^-1)��" << matMxhxh(12, 0) << std::endl;

		//��������ֵ
		outfile << "======== ��������ֵ(pix) ========" << std::endl;
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
		std::cout << "����ʧ�ܣ�" << "������" << i + 1 << "��" << std::endl;
	}

	outfile.close();
}


