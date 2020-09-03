#ifndef SRC_PATH_POINT_H
#define SRC_PATH_POINT_H

struct PathPoint
{
    PathPoint(float, float); 
    float x;		// Corresponding to x position on map
    float y;		// Corresponding to y position on map
    float velocity;	// Corresponding to calculated reference velocity?
    float radius;	// Corresponding to radius of curvature

};


#endif // SRC_PATH_POINT_H
