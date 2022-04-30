#ifndef __MATRIX_TYPES_H
#define __MATRIX_TYPES_H

struct Point_
{
	float x;
	float y;
};
typedef struct Point_ Point;

struct Pose_
{
	float x;
	float y;
	float theta;
};
typedef struct Pose_ Pose;

struct Covarince_
{
	float a1;
	float a2;
	float a3;
	float a4;
};
typedef struct Covarince_ Covarince;

struct Hessian_
{
	float a[3][3];
};
typedef struct Hessian_ Hessian;

struct Dtr_
{
	float a[3];
};
typedef struct Dtr_ Dtr;




#endif
