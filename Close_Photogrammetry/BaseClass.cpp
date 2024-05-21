#include "BaseClass.h"

void BaseClass::leftHand2RightHand(std::map<int, Eigen::Vector3d>& objCoor)
{
    double x, y, z;
    for (auto it = objCoor.begin(); it != objCoor.end(); it++)
    {
        x = it->second.x();
        y = it->second.y();
        z = it->second.z();

        it->second.x() = y;
        it->second.y() = z;
        it->second.z() = -x;
    }
}

void BaseClass::rightHand2LeftHand(std::map<int, Eigen::Vector3d>& objCoor)
{
    double x, y, z;
    for (auto it = objCoor.begin(); it != objCoor.end(); it++)
    {
        x = it->second.x();
        y = it->second.y();
        z = it->second.z();

        it->second.x() = -z;
        it->second.y() = x;
        it->second.z() = y;
    }
}
