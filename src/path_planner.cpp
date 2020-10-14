#include "path_planner.h"
#include "path_point.h"
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <utility>
#include <string>

PathPlanner::PathPlanner(float car_x, float car_y, const std::vector<Cone> &cones, bool const_velocity, float v_max, float v_const, float max_f_gain)
    : const_velocity(const_velocity), v_max(v_max), v_const(v_const), f_gain(max_f_gain) 
{
    int num_l = 0;
    int num_r = 0;

	raw_cones.reserve(1000);
	l_cones_to_add.reserve(1000);
	r_cones_to_add.reserve(1000);

    // Add to list of raw_cones so that references can be made to be amended
    // Need to change to take in points not cones, these cones will then be converted to cones 
	addCones(cones);
    
    // Add the car position to the centre points 
    PathPoint car_pos = PathPoint(car_x, car_y);
    centre_points.push_back(car_pos);
	centre_points.push_back(centralizeTimingCones());

    // Sort by distance to car 
    sortConesByDist(centre_points.back(), centre_points.back());
	
    // Add CLOSEST cones to vector of known cones
    left_cones.push_back(l_cones_to_add.front());
    right_cones.push_back(r_cones_to_add.front());

    // Clear pointers and reset l/r_cones_to_add
	removeFirstPtr(l_cones_to_add);
	removeFirstPtr(r_cones_to_add);
    l_cones_sorted = false;
    r_cones_sorted = false;
    
    // Re-sort by distance to closest cone
    sortConesByDist(left_cones.front()->position, right_cones.front()->position);
    popConesToAdd();
    addFirstCentrePoints();
}

void PathPlanner::update(const std::vector<Cone> &new_cones, const float car_x, const float car_y,
						 std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V)
{
	if (left_start_zone)
	{
		// join track if feasible
		if (joinFeasible(car_x, car_y))
		{
			centre_points.push_back(centre_points.front());
			reached_end_zone = true;
		}
	}

    if (!reached_end_zone)
    {
	    addCones(new_cones);
	    sortConesByDist(left_cones.back()->position, right_cones.back()->position);
	    popConesToAdd();
	    addCentrePoints(car_x, car_y);
		addVelocityPoints();
    }

	returnResult(X, Y, V);

	std::cout << "num cps: " << centre_points.size() << std::endl;
	std::cout << std::endl;
}

bool PathPlanner::joinFeasible(const float &car_x, const float &car_y)
{
	/*
	Two conditions:
		- centre point feasible
		- car within set distance
	*/
}

void PathPlanner::returnResult(std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V)
{
    for (auto &e: centre_points)
	{
		X.push_back(e.x); 
		Y.push_back(e.y); 
		V.push_back(e.velocity);
	}
}

void PathPlanner::addVelocityPoints()
{
    if (!const_velocity)
    {
		if (centre_points.size() >= 3)
		{
			for (size_t i = 1; i < centre_points.size() - 1; i++)
			{
				centre_points[i].radius = calcRadius(centre_points[i - 1], 
									centre_points[i], 
									centre_points[i + 1]);

			}
			centre_points.front().velocity = std::min(f_gain * sqrt(centre_points[1].radius), v_max);
			centre_points.back().velocity = 0;

			if (centre_points.size() == 3)
			{
				centre_points[1].velocity = std::min(f_gain * sqrt(centre_points[0].radius), v_max);
			}
			else
			{
				centre_points[1].velocity = std::min(f_gain * (sqrt(centre_points[0].radius) +
															   sqrt(centre_points[1].radius)) / 2, v_max);

				centre_points.end()[-2].velocity = std::min(f_gain * (sqrt(centre_points.back().radius) 
										+ sqrt(centre_points.end()[-2].radius)) / 2, v_max);

				for (size_t i = 2; i < centre_points.size(); i++)
				{
					centre_points[i].velocity = std::min(f_gain * (sqrt(centre_points[i - 2].radius) +
										sqrt(centre_points[i - 1].radius) + 
										sqrt(centre_points[i].radius)) / 3, v_max); 
				}
			}
		}

		else
		{
			for (auto &point: centre_points)
			{
				point.velocity = v_const;
			}
		}
    }
    else
	{
		for (auto &point: centre_points)
		{
			point.velocity = v_const;
		}
	}
}

void PathPlanner::shutdown()
{
    left_cones.clear();
    right_cones.clear();
    timing_cones.clear();
    l_cones_to_add.clear();
    r_cones_to_add.clear();
}

float PathPlanner::calcAngle(const PathPoint &A, const PathPoint &B, const PathPoint &C)
{
	float cb_x = B.x - C.x;
	float cb_y = B.y - C.y;
	float ca_x = B.x - A.x;
	float ca_y = B.y - A.y;

	float angle = (atan2(cb_x, cb_y) - atan2(ca_x, ca_y)) * 180 / PI;

	return angle;
}

float PathPlanner::calcRelativeAngle(const PathPoint &p1, const PathPoint &p2)
{
	const float angle = atan2(p2.y - p1.y, p2.x - p1.x);

	return angle;
}

float PathPlanner::calcRadius(const PathPoint &A, const PathPoint &B, const PathPoint &C)
{
    std::pair<float, float> AB_mid {(A.x + B.x) / 2, (A.y + B.y) / 2};
    std::pair<float, float> BC_mid {(B.x + C.x) / 2, (B.y + C.y) / 2};
    std::pair<float, float> AC_mid {(A.x + C.x) / 2, (A.y + C.y) / 2};

    float AB_m = (A.y - B.y) / (A.x - B.x);
    float BC_m = (C.y - B.y) / (C.x - B.x);
    float AC_m = (C.y - A.y) / (C.x - A.x);

    float AB_mp = -1 / AB_m;
    float BC_mp = -1 / BC_m;
    float AC_mp = -1 / AC_m;

    float AB_c = AB_mid.second - AB_mp * AB_mid.first;
    float BC_c = BC_mid.second - BC_mp * BC_mid.first;
    float AC_c = AC_mid.second - AC_mp * AC_mid.first;

    float x_AB_BC = (BC_c - AB_c) / (AB_mp - BC_mp);
    float x_AB_AC = (AC_c - AB_c) / (AB_mp - AC_mp);
    float x_BC_AC = (AC_c - BC_c) / (BC_mp - AC_mp);

    float y_AB_BC = AB_mp * x_AB_BC + AB_c;
    float y_AB_AC = AB_mp * x_AB_AC + AB_c;
    float y_BC_AC = BC_mp * x_BC_AC + BC_c;

    float x_int = (x_AB_BC + x_AB_AC + x_BC_AC) / 3;
    float y_int = (y_AB_BC + y_AB_AC + y_BC_AC) / 3;

    float rad_A = sqrt(pow(A.y - y_int, 2) + pow(A.x - x_int, 2)); 
    float rad_B = sqrt(pow(B.y - y_int, 2) + pow(B.x - x_int, 2)); 
    float rad_C = sqrt(pow(C.y - y_int, 2) + pow(C.x - x_int, 2)); 
    
    return (rad_A + rad_B + rad_C) / 3;
}

PathPoint PathPlanner::generateCentrePoint(const Cone* cone_one, const Cone* cone_two, bool& feasible)
{
	PathPoint midpoint(
		(cone_one->position.x + cone_two->position.x) / 2,
		(cone_one->position.y + cone_two->position.y) / 2
	);

	float dist_back = calcDist(centre_points.back(), midpoint);
	float angle = abs(calcAngle(*(centre_points.end() - 2), centre_points.back(), midpoint));

	if (angle > 95 && angle < 265 && dist_back > 0.5 && dist_back < 12)
	{
		std::cout << "Accepted point: " << dist_back << ' ' << angle << std::endl;
		feasible = true;
	}
	else
	{
		std::cout << "Rejected point: " << dist_back << ' ' << angle << std::endl;
		feasible = false;
	}

	return midpoint;
}

void PathPlanner::addCentrePoints(const float &car_x, const float &car_y)
{
	bool feasible;
	PathPoint cp;
	int opp_idx;
	
	if (centre_points.size() > 2)
	{
		for (int i = 0; i < left_cones.size(); i++)
		{
			if (!left_cones[i]->mapped && left_cones[i]->times_checked < 50)
			{
				opp_idx = findOppositeClosest(*left_cones[i], right_cones);
				if (opp_idx != -1)
				{
					left_cones[i]->times_checked++;
					right_cones[opp_idx]->times_checked++;
					feasible = false;
					cp = generateCentrePoint(left_cones[i], right_cones[opp_idx], feasible);

					if (feasible)
					{
						centre_points.push_back(cp);	
						left_cones[i]->mapped = true;
					}
				}
			}
		}

		for (int i = 0; i < right_cones.size(); i++)
		{
			if (!right_cones[i]->mapped && right_cones[i]->times_checked < 50)	
			{
				opp_idx = findOppositeClosest(*right_cones[i], left_cones);
				if (opp_idx != -1)
				{
					feasible = false;
					cp = generateCentrePoint(right_cones[i], left_cones[opp_idx], feasible);
					left_cones[opp_idx]->times_checked++;
					right_cones[i]->times_checked++;

					if (feasible)
					{
						centre_points.push_back(cp);
						right_cones[i]->mapped = true;
					}
				}
			}
		}
	}
}

void PathPlanner::addCones(const std::vector<Cone> &new_cones)
{
	std::vector<Cone> b_temp;
	std::vector<Cone> y_temp;
	std::vector<Cone> r_temp;
	b_temp.reserve(new_cones.size());
	y_temp.reserve(new_cones.size());
	r_temp.reserve(new_cones.size());

	for (auto &cone: new_cones)
	{
		if (cone.colour == 'b')
		{
			b_temp.push_back(cone);
		}
		else if (cone.colour == 'y')
		{
			y_temp.push_back(cone);
		}
		else
		{
			r_temp.push_back(cone);
		}
	}

	int test_l = 0;
	int test_r = 0;

	for (int i = left_cones.size(); i < b_temp.size(); i++)
	{
		if (left_cones.size() + l_cones_to_add.size() < b_temp.size())
		{
			test_l++;
			raw_cones.push_back(b_temp[i]);
			l_cones_to_add.push_back(&raw_cones.back());
			l_cones_sorted = false;
		}
	}

	for (int i = right_cones.size(); i < y_temp.size(); i++)
	{
		if (right_cones.size() + r_cones_to_add.size() < y_temp.size())
		{
			test_r++;
			raw_cones.push_back(y_temp[i]);
			r_cones_to_add.push_back(&raw_cones.back());
			r_cones_sorted = false;
		}			
	}

	for (auto &cone: r_temp)
	{
		if (timing_cones.size() < r_temp.size())
		{
			raw_cones.push_back(cone);
			timing_cones.push_back(&raw_cones.back());
		}
	}
}

void PathPlanner::addFirstCentrePoints()
{
    size_t n = std::min(left_cones.size(), right_cones.size());		
    int closest_opp_idx;
    float centre_x, centre_y;

    for (size_t i = 0; i < n; i++)
    {
	    if (!left_cones[i]->mapped)
	    {
		    closest_opp_idx = findOppositeClosest(*left_cones[i], right_cones);
		    if (closest_opp_idx != -1)
		    {
			    centre_x = (right_cones[closest_opp_idx]->position.x + left_cones[i]->position.x) / 2;
			    centre_y = (right_cones[closest_opp_idx]->position.y + left_cones[i]->position.y) / 2;
			    centre_points.push_back(PathPoint(centre_x, centre_y));
			    left_cones[i]->mapped = true;
			    right_cones[closest_opp_idx]->mapped = true;
		    }
	    }
    }
}

PathPoint PathPlanner::centralizeTimingCones()
{
	// Get average x and y position of all timing cones
	PathPoint avg_point(0, 0);
	float pcp_dist;

	for (int i = 0; i < timing_cones.size(); i++)
	{
		avg_point.x += timing_cones[i]->position.x; // summation of x positions
		avg_point.y += timing_cones[i]->position.y; // summation of y positions
	} 

	avg_point.x = avg_point.x / timing_cones.size(); // avg x dist
	avg_point.y = avg_point.y / timing_cones.size(); // avg y dist

	// Calc distance to previous centrepoint
	pcp_dist = calcDist(centre_points.front(), avg_point);

	if (pcp_dist < 4)
	{
		std::cout << "Average timing cone position calculated" << std::endl;
		return avg_point;
	}
}

int PathPlanner::findOppositeClosest(const Cone &cone, const std::vector<Cone*> &cones)
{
	float min_dist = 12;
	float dist;
	int index = -1;
	int i = 0;

	for (auto it = cones.begin(); it != cones.end(); it++)
	{
	    dist = calcDist(cone.position, (*it)->position);

	    if (dist < min_dist)
	    {
			min_dist = dist;
			index = i;
	    }
	    i++;
	}
	return index;
}

void PathPlanner::popConesToAdd()
{
    double dist;

    // NEEDS TO BE DISTANCE TO THE LAST CONE ON UPDATE, NOT DISTANCE TO THE FIRST IN LCONESTOADD

    while (!l_cones_to_add.empty())
    {
		if (left_cones.empty()) // upon init 
		{
			dist = calcDist(l_cones_to_add[0]->position, l_cones_to_add[1]->position);
		}
		else 
		{
			dist = calcDist(left_cones.back()->position, l_cones_to_add.front()->position);
		}
		if (dist < 10)
		{
			left_cones.push_back(l_cones_to_add.front());
		}
		removeFirstPtr(l_cones_to_add);
	}

	while (!r_cones_to_add.empty())
	{ 
		if (right_cones.empty()) // upon init
		{
			dist = calcDist(r_cones_to_add[0]->position, r_cones_to_add[1]->position);
		}
		else 
		{
			dist = calcDist(right_cones.back()->position, r_cones_to_add.front()->position);
		}
		if (dist < 10)
		{
			right_cones.push_back(r_cones_to_add.front());
		}
		removeFirstPtr(r_cones_to_add);
    }
}

void PathPlanner::removeFirstPtr(std::vector<Cone*>& cone_vec)
{
    if (cone_vec.size() > 0 && cone_vec.front() != NULL)
    {
		cone_vec.erase(cone_vec.begin());
    }
}

void PathPlanner::sortConesByDist(const PathPoint &left, const PathPoint &right)
{
	// Assign distance Cone objects on left
    for (auto &cone: l_cones_to_add)
    {cone->dist = calcDist(left, cone->position);}

	// Assign distance to Cone objects on right
    for (auto &cone: r_cones_to_add) 
    {cone->dist = calcDist(right, cone->position);}

	// nlogn sort both cones_to_add vectors
    sort(l_cones_to_add.begin(), l_cones_to_add.end(), compareConeDist);
    sort(r_cones_to_add.begin(), r_cones_to_add.end(), compareConeDist);

    l_cones_sorted = true;
    r_cones_sorted = true;
}

float PathPlanner::calcDist(const PathPoint &p1, const PathPoint &p2)
{
    float x_dist = pow(p2.x - p1.x, 2);
    float y_dist = pow(p2.y - p1.y, 2);

    return sqrt(x_dist + y_dist);
}

void PathPlanner::resetTempConeVectors()
{
	l_cones_to_add.clear();
	r_cones_to_add.clear();
	l_cones_sorted = false;
	r_cones_sorted = false;
}

bool PathPlanner::compareConeDist(Cone* const &cone_one, Cone* const &cone_two)
{
    return cone_one->dist < cone_two->dist;
}

     
