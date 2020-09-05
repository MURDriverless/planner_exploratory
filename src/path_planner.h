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
    PathPlanner(float, float, const std::vector<Cone>&, bool, float, float, float);
    std::vector<Cone> raw_cones;
    std::vector<Cone*> left_cones;		// Cones on left-side of track
    std::vector<Cone*> right_cones;		// Cones on right-side of track
    std::vector<PathPoint> centre_points;
    std::vector<Cone*> timing_cones;		// Cones for timing? TO ASK
    std::vector<PathPoint> path;		// PathCoordinates corresponding to planned path
    std::vector<Cone*> l_cones_to_add; 
    std::vector<Cone*> r_cones_to_add; 
    std::vector<PathPoint> final_points;
    std::vector<PathPoint> velocity_points;
    std::vector<float> radius_points;
    std::vector<PathPoint> centre_spline;
    bool l_cones_sorted = false;
    bool r_cones_sorted = false;
    bool set_final_points = false;
    bool left_start_zone = false;
    bool reached_end_zone = false;
    bool const_velocity;
    size_t l_cone_index = 0;
    size_t r_cone_index = 0;
    float v_max;
    float v_const;
    float f_gain;

    void resetTempConeVectors();
    std::vector<float> update(const std::vector<Cone>&, const float, const float);

private:
    int findOppositeClosest(const Cone&, const std::vector<Cone*>&);
    void addFirstCentrePoints();
    void addCentrePoints();
    void sortConesByDist(const PathPoint&, const PathPoint&);
    static bool compareConeDist(Cone* const&, Cone* const&);
    void popConesToAdd();
    void calcSpline();
    void addCones(const std::vector<Cone>&);
    void addVelocityPoints();
    float calcRadius(const PathPoint&, const PathPoint&, const PathPoint&);
    float calcDist(const PathPoint&, const PathPoint&);
    void removeFirstPtr(std::vector<Cone*>&);
};

#endif // INCLUDE_PATH_PLANNER_H
