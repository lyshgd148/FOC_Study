#include "serial.h"

void Serial_DataProcess(uint8_t *data)
{

    char *key = strtok(data, ":");
    char *value = strtok(NULL, ":");
    if (strcmp(key, "p1") == 0)
        traj.pos1 = atof(value);
    else if (strcmp(key, "p2") == 0)
        traj.pos2 = atof(value);
    else if (strcmp(key, "v") == 0)
        traj.maxVel = atof(value);
    else if (strcmp(key, "a") == 0)
        traj.maxAcc = atof(value);
}

// {
//     char *data[] = {"p1:130", "p2:215", "v:6.2", "a:2.7"};
//     int p1, p2;
//     float v, a;

//     for(int i=0; i<4; i++)
//     {
//         char *key = strtok(data[i], ":");
//         char *value = strtok(NULL, ":");

//         if(strcmp(key, "p1") == 0)
//             p1 = atoi(value);
//         else if(strcmp(key, "p2") == 0)
//             p2 = atoi(value);
//         else if(strcmp(key, "v") == 0)
//             v = atof(value);
//         else if(strcmp(key, "a") == 0)
//             a = atof(value);
//     }

//     printf("p1=%d, p2=%d, v=%.2f, a=%.2f\n", p1, p2, v, a);
//     return 0;
// }