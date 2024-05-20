#include "BaseClass.h"

void BaseClass::getImageCoor(const std::string& filePath, std::map<int, Eigen::Vector2d>& imageCoor)
{
    int w = 4272;
    int h = 2848;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cout << "Error opening file: " << filePath << std::endl;
    }
    else {
        int ID;
        double x, y;
        std::string line;
        std::getline(file, line);// ¶ÁÈ¡Ê×ÐÐ
        while (!file.eof()) {
            file >> ID >> x >> y;
            if (ID >= 100)
            {
                imageCoor[ID] = Eigen::Vector2d(x, h - 1 - y);
            }
        }
        file.close();
    }
}

void BaseClass::getObjCoor(const std::string& filePath, std::map<int, Eigen::Vector3d>& objCoor)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cout << "Error opening file: " << filePath << std::endl;
    }
    else {
        int ID, num;
        double x, y, z;
        file >> num;
        for (int i = 0; i < num; i++) {
            file >> ID >> x >> y >> z;
            objCoor[ID] = Eigen::Vector3d(x, y, z);
        }
        file.close();
    }
}

void BaseClass::leftHand2RightHand(const std::map<int, Eigen::Vector3d>& origin, std::map<int, Eigen::Vector3d>& result)
{
    double x, y, z;
    for (auto it = origin.begin(); it != origin.end(); it++)
    {
        x = it->second.x();
        y = it->second.y();
        z = it->second.z();
        result[it->first] = Eigen::Vector3d(y, z, -x);
    }
}

void BaseClass::rightHand2LeftHand(const std::map<int, Eigen::Vector3d>& origin, std::map<int, Eigen::Vector3d>& result)
{
    double x, y, z;
    for (auto it = origin.begin(); it != origin.end(); it++)
    {
        x = it->second.x();
        y = it->second.y();
        z = it->second.z();
        result[it->first] = Eigen::Vector3d(-z, x, y);
    }
}
