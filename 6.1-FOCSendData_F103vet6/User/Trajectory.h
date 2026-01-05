#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include <stdbool.h>

struct Struct_Trajectory
{
	float pos1;
	float pos2;
	float maxVel;
	float maxAcc;

	float t1;
	float t2;
	float t3;

	float position;
	float velocity;
	float acceleration;

	bool IsTriangle;
	float dir;
};

extern struct Struct_Trajectory traj;

void Traj_Init(float MaxAcc, float MaxVel, float Position1, float Position2, struct Struct_Trajectory *traj);
void GetP_V(float time, struct Struct_Trajectory *traj);
#endif
