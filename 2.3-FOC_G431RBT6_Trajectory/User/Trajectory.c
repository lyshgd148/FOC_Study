#include "Trajectory.h"
#include "math.h"

struct Struct_Trajectory traj;

void GetP_V(float time, struct Struct_Trajectory *traj)
{
    if (time < traj->t1)
    {
        traj->position = traj->pos1 + (0.5 * time * time) * traj->maxAcc * traj->dir;
        traj->velocity = time * traj->maxAcc * traj->dir;
    }
    else if (time < traj->t2)
    {
        // traj->position = traj->pos1 + (0.5 * traj->t1 * traj->t1 * traj->maxAcc + (time - traj->t1) * traj->t1 * traj->maxAcc) * traj->dir;
        traj->position = traj->pos1 + (0.5 * traj->t1 * traj->t1 * traj->maxAcc + (time - traj->t1) * traj->maxVel) * traj->dir; // 能够进入这个判断条件一定是梯形
        traj->velocity = traj->t1 * traj->maxAcc * traj->dir;
    }
    else if (time <= traj->t3)
    {
        traj->position = traj->pos2 - 0.5 * (traj->t3 - time) * (traj->t3 - time) * traj->maxAcc * traj->dir;
        traj->velocity = (traj->t3 - time) * traj->maxAcc * traj->dir;
    }
    else
    {
        traj->position = traj->pos2;
        traj->velocity = 0;
    }
}

void Traj_Init(float MaxAcc, float MaxVel, float Position1, float Position2, struct Struct_Trajectory *traj) // 速度 加速度 是绝对值
{
    traj->maxAcc = MaxAcc;
    traj->maxVel = MaxVel;
    traj->pos1 = Position1;
    traj->pos2 = Position2;
    traj->dir = Position2 > Position1 ? 1 : -1;

    float d = (Position2 - Position1) * traj->dir;
    float at = MaxVel / MaxAcc;

    if ((MaxAcc * at * at) >= d)
    {
        traj->t1 = sqrt(d / MaxAcc);
        traj->t2 = traj->t1;
        traj->t3 = 2 * traj->t1;
        traj->IsTriangle = true;
    }
    else
    {
        traj->t1 = at;
        traj->t2 = at + (d - MaxVel * at) / MaxVel;
        traj->t3 = traj->t2 + at;
        traj->IsTriangle = false;
    }
}