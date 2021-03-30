#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <algorithm> 

struct Point {
    double x;
    double y;
    double z;
};


// elbow and hand positions specified in mm
struct RobotArm {
    Point elbow;
    Point hand;
};

struct Vector{
        double x,y,z;
};


Vector operator - (const Vector v1, const Vector v2 )
{
    Vector result;
    result.x = v1.x-v2.x;
    result.y = v1.y-v2.y;
    result.z = v1.z-v2.z;

    return result;
}


Vector operator + (const Vector v1, const Vector v2 )
{
    Vector result;
    result.x = v1.x+v2.x;
    result.y = v1.y+v2.y;
    result.z = v1.z+v2.z;

    return result;
}


Vector operator * (double v1, const Vector v2 )
{
    Vector result;
    result.x = v1*(v2.x);
    result.y = v1*(v2.y);
    result.z = v1*(v2.y);

    return result;
}
