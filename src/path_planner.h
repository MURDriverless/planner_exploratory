#ifndef SRC_PATH_PLANNER_H
#define SRC_PATH_PLANNER_H

#include <vector>
#include <cstdint>
#include <memory>
#include "cone.h"
#include "path_point.h"

class PathPlanner 
{
public:
    PathPlanner(float, float, std::vector<Cone>, bool, uint8_t, uint8_t, uint8_t);
    std::vector<Cone> raw_cones;
    std::vector<Cone*> left_cones;		// Cones on left-side of track
    std::vector<Cone*> right_cones;	// Cones on right-side of track
    std::vector<PathPoint> centre_points;
    std::vector<Cone*> timing_cones;	// Cones for timing? TO ASK
    std::vector<PathPoint> path;						// PathCoordinates corresponding to planned path
    std::vector<Cone*> l_cones_to_add; 
    std::vector<Cone*> r_cones_to_add; 
    PathPoint final_points(float, float);
    std::vector<PathPoint> centre_spline;
    bool l_cones_sorted = false;
    bool r_cones_sorted = false;
    bool set_final_points = false;
    bool left_start_zone = false;
    bool reached_end_zone = false;
    bool const_velocity;
    uint8_t l_cone_index = 1;
    uint8_t r_cone_index = 1;
    uint8_t v_max;
    uint8_t v_const;
    uint8_t max_f_gain;


private:
    int findOppositeClosest(const Cone&, const std::vector<Cone*>&);
    void addFirstCentrePoints();
    void sortConesByDist(PathPoint&, PathPoint&);
    static bool compareConeDist(Cone* const&, Cone* const&);
    void resetTempConeVectors();
    void popConesToAdd();
    void calcSpline();
    void update(std::vector<Cone>&);
	
};

#endif // INCLUDE_PATH_PLANNER_H
