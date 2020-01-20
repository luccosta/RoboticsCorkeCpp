#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using namespace std;

struct Coordinate
{
	double x, y, z;

	Coordinate(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

struct Rotation
{
	double r, p, y;

	Rotation(double roll, double pitch, double yall)
	{
		r = roll;
		p = pitch;
		y = yall;
	}
};

#define D 3

Matrix3d RotationMatrix(float, float, float);
Matrix4d HomogeneousTransformationMatrix(Coordinate, Rotation);
Matrix4d TranslationMatrix(Coordinate);

int main()
{
	Matrix3d R;
	Matrix4d T;

	T = HomogeneousTransformationMatrix(Coordinate(4,5,6), Rotation(0, 0, 0));

	cout << T << endl;
}

Matrix3d RotationX(float thetax)
{
	Matrix3d Rx;

	Rx << 1, 0, 0,
		  0, cos(thetax), -sin(thetax),
		  0, sin(thetax), sin(thetax);
	
	return Rx;
}

Matrix3d RotationY(float thetay)
{
	Matrix3d Ry;

	Ry << cos(thetay), 0, sin(thetay),
		  0, 1, 0,
		  -sin(thetay), 0, cos(thetay);
	
	return Ry;
}

Matrix3d RotationZ(float thetaz)
{
	Matrix3d Rz;

	Rz << cos(thetaz), -sin(thetaz), 0,
		  sin(thetaz), cos(thetaz), 0,
		  0, 0, 1;
	
	return Rz;
}

Matrix3d RotationMatrix(float thetaz1, float thetay, float thetaz2) // Angle in degrees
{
	return RotationZ(thetaz1)*RotationY(thetay)*RotationZ(thetaz2);
}

Matrix4d HomogeneousTransformationMatrix(Coordinate origin, Rotation rot)
{
	Matrix4d T;

	T.block(0, 0, 3, 3) << RotationMatrix(rot.r, rot.p, rot.y);
	T.block(0, 3, 3, 1) << origin.x, origin.y, origin.z;
	T.block(3, 0, 1, 3) << MatrixXd::Zero(1,3);
	T.block(3, 3, 1, 1) << MatrixXd::Identity(1,1);

	return T;
}

Matrix4d TranslationMatrix(Coordinate origin)
{
	return HomogeneousTransformationMatrix(origin, Rotation(0,0,0));
}