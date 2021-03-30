#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <algorithm> 
#include "structures.h"



#define dot(u,v)  ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define norm(v)    sqrt(dot(v,v))  // norm = length of vector
#define d(u,v)    norm(u-v)      // distance = norm of difference
#define abs(x)    ((x) >= 0 ? (x) : -(x))  // absolute value

double smallest(double a, double b, double c, double d){
    return std::min(a, std::min(b, std::min(c, d)));
}



double Norm(Point p1, Point p2){

    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}


Point PointToSegInter(Point pt, Point p1, Point p2){
    Point foot;

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    double epsilon = 0.00001;
    if(abs(dx) < epsilon && abs(dy) < epsilon && abs(dz) < epsilon)
    {
        foot = p1;
        return foot;
    }

    double u = (pt.x - p1.x)*(p1.x - p2.x) +
        (pt.y - p1.y)*(p1.y - p2.y) + (pt.z - p1.z)*(p1.z - p2.z);
    u = u/((dx*dx)+(dy*dy)+(dz*dz));

    foot.x = p1.x + u*dx;
    foot.y = p1.y + u*dy;
    foot.z = p1.z + u*dz;
    return foot;
}


double PointToSegDist(Point pt, Point p1, Point p2){
    double  x1 = p1.x;
    double 	y1 = p1.y;
    double 	z1 = p1.z;
    double 	x2 = p2.x;
    double 	y2 = p2.y;
    double 	z2 = p2.z;
    double 	x = pt.x;
    double 	y = pt.y;
    double 	z = pt.z;
    double dotproduct = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1) + (z2 - z1) * (z - z1);
    double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1);
    double shortest_dist = 0;
    double epsilon = 0.0001;

    // PA
    if (dotproduct <= 0) {
        shortest_dist = Norm(pt,p1);
    }
    // PB
    else if (dotproduct >= d2) {
        shortest_dist = Norm(pt,p2);
    }
    // Perpindicular Distance PM
    else {

        Point p_inter = PointToSegInter(pt, p1, p2);
        shortest_dist = Norm(pt, p_inter);
    }
    return shortest_dist;
}


double SegToSegDist(Point p1, Point p2, Point p3, Point p4){

    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dz1 = p2.z - p1.z;
    double dx2 = p3.x - p1.x;
    double dy2 = p3.y - p1.y;
    double dz2 = p3.z - p1.z;
    double dx3 = p4.x - p1.x;
    double dy3 = p4.y - p1.y;
    double dz3 = p4.z - p1.z;
    double dx4 = p4.x - p3.x;
    double dy4 = p4.y - p3.y;
    double dz4 = p4.z - p3.z;
    double epsilon = 0.001;
    double best_dist = 0;

    double dist_1 = PointToSegDist(p1, p3, p4);
    double dist_2 = PointToSegDist(p2, p3, p4);
    double dist_3 = PointToSegDist(p3, p1, p2);
    double dist_4 = PointToSegDist(p4, p1, p2);

    double ss = (dy1*dx2 -dy2*dx1)/(dy4*dx1 - dy1*dx4);

      if(abs(dx1*dy2*dz3+dx2*dy3*dz1+dx3*dy1*dz2-dx3*dy2*dz1-dx1*dy3*dz2-dx2*dy1*dz3) > epsilon){ // scew

        Point u, v, w;
        u.x = -dx1;
        u.y = -dy1;
        u.z = -dz1;
        v.x = dx4;
        v.y = dy4;
        v.z = dz4;
        w.x = -dx2;
        w.y = -dy2;
        w.z = -dz2;
        double    a = dot(u,u);
        double    b = dot(u,v);
        double    c = dot(v,v);
        double    d = dot(u,w);
        double    e = dot(v,w);
        double    D = a*c - b*b;
        double    sc, tc;
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
        Point dP, point_on_p1p2, point_on_p3p4;
        dP.x = w.x + sc * u.x - tc * v.x;
        dP.y = w.y + sc * u.y - tc * v.y;
        dP.z = w.z + sc * u.z - tc * v.z;
        point_on_p1p2.x = p1.x + sc * u.x;
        point_on_p1p2.y = p1.y + sc * u.y;
        point_on_p1p2.z = p1.z + sc * u.z;
        point_on_p3p4.x = p3.x + tc * v.x;
        point_on_p3p4.y = p3.y + tc * v.y;
        point_on_p3p4.z = p3.z + tc * v.z;

        double perp_dist = norm(dP);

            if (abs(Norm(p1,p2) - Norm(p1,point_on_p1p2) - Norm(p2,point_on_p1p2)) < epsilon && abs(Norm(p3,p4) - Norm(p3,point_on_p3p4) - Norm(p4,point_on_p3p4)) < epsilon) { // on two segments

                best_dist = perp_dist;;
            }
            else { // not on two segments

                best_dist = smallest(dist_1, dist_2, dist_3, dist_4);
            }
        }

     else {

       if(abs(dx2/dx1 - dy2/dy1) < epsilon && abs(dy2/dy1 - dz2/dz1) < epsilon) { // parallel;
            best_dist = std::min(dist_1, dist_2);
        }

       else {   // intersect

            Point intersection = {};
            if (p1.x == 0 && dx1 == 0 or p3.x == 0 && dx4 == 0) {
                intersection.x == 0;
            } else {
                intersection.x = p3.x + dx4*ss;
            }

            if  (p1.y == 0 && dy1 == 0 or p3.y == 0 && dy4 == 0) {
                intersection.y == 0;
            } else {
                intersection.y = p3.y + dy4*ss;
            }
            if  (p1.z == 0 && dz1 == 0 or p3.z == 0 && dz4 == 0) {
                intersection.z == 0;
            } else {
                intersection.z = p3.z + dz4*ss;
            }

            if (abs(Norm(p1,p2) - Norm(p1,intersection) - Norm(p2,intersection)) < 0.001 && abs(Norm(p3,p4) - Norm(p3,intersection) - Norm(p4,intersection)) < 0.001) {    // segment intersect

                best_dist = 0;
            }
            else {
                best_dist = smallest(dist_1, dist_2, dist_3, dist_4);
            }
        return best_dist;
    }
}
}
