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

    // Add to list of raw_cones so that references can be made to be amended
    // Need to change to take in points not cones, these cones will then be converted to cones 
    for (auto& cone: cones) {raw_cones.push_back(cone);}

    // Prepare initial cones to be added to the path planner 
    for (auto& cone: raw_cones) 
    {
	if (cone.colour == 'b')
	{
	    num_l++;
	    l_cones_to_add.push_back(&cone);
	    l_cones_sorted = false;
	}
	else if (cone.colour == 'y')
	{
	    num_r++;
	    r_cones_to_add.push_back(&cone);
	    r_cones_sorted = false;
	}
	else if (cone.colour == 'r')
	{
	   timing_cones.push_back(&cone); 
	}
    }
    
    // Add the car position to the centre points (THIS HAS BEEN TRUNCATED - confirm with Alex)
    PathPoint car_pos = PathPoint(car_x, car_y);
    centre_points.push_back(car_pos);

    // Sort by distance to car 
    sortConesByDist(car_pos, car_pos);
	
    // Add CLOSEST cones to vector of known cones
    left_cones.push_back(l_cones_to_add.front());
    right_cones.push_back(r_cones_to_add.front());

    // Clear pointers and reset l/r_cones_to_add
    l_cones_to_add.erase(l_cones_to_add.begin());
    r_cones_to_add.erase(r_cones_to_add.begin());
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
	// Add new cones to local list of cones
    if (!reached_end_zone)
    {
	    addCones(new_cones);
	    sortConesByDist(left_cones.back()->position, right_cones.back()->position);
	    popConesToAdd();
	    addCentrePoints(car_x, car_y);
    }

    if (!set_final_points && timing_cones.size() > 2)
	// UNTESTED
    {
		float delta_x = timing_cones[0]->position.x - timing_cones[1]->position.x;
		float delta_y = timing_cones[0]->position.y - timing_cones[1]->position.y;
		bool comp_result;

	if (delta_x == 0.0)
	{
	    comp_result = centre_points[0].x < timing_cones[0]->position.x;
	    for (size_t i = final_points.size() - 1; i < centre_points.size(); i++)
	    {
			if ((final_points[i].x < timing_cones[0]->position.x) == comp_result)
			{
				final_points.push_back(centre_points[i]);
			}
			else
			{
				final_points.push_back(PathPoint(timing_cones[0]->position.x,
												(timing_cones[0]->position.y + timing_cones[1]->position.y) / 2));

				final_points.push_back(centre_points[i]);
				set_final_points = true;
			}
	    }
	}
	else
	    // TESTED
	{
	    float m = delta_y / delta_x;
	    float c = timing_cones[0]->position.y - m * timing_cones[0]->position.x;
	    comp_result = centre_points[0].y < m * centre_points[0].x + c;

	    for (size_t i = final_points.size(); i < centre_points.size(); i++)
	    {
			if ((centre_points[i].y < m * centre_points[i].x + c) == comp_result)
			{
				final_points.push_back(centre_points[i]);
			}
			else
			{
				final_points.push_back(PathPoint((timing_cones[0]->position.x + timing_cones[1]->position.x) / 2,
												 (timing_cones[0]->position.y + timing_cones[1]->position.y) / 2));
				final_points.push_back(centre_points[i]);
				set_final_points = true;
			}
	    }
	}

	if (!reached_end_zone)
	{
	    if (!left_start_zone)
	    {
		float dist_c = calcDist(centre_points.front(), PathPoint(car_x, car_y));
		if (dist_c >= 10)
		{
		    left_start_zone = true;
		}
	    }
	    else
	    {
			float dist_l = calcDist(left_cones.front()->position, right_cones.back()->position);
			float dist_r = calcDist(right_cones.front()->position, right_cones.back()->position);

		if (dist_l <= 5 && dist_r <= 5)
		{
		    reached_end_zone = true;
		    for (auto &fp: final_points)
		    {
				centre_points.push_back(fp);
		    }
		}
	    }
	}
    }
    addVelocityPoints();
    returnResult(X, Y, V);
}

void PathPlanner::returnResult(std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V)
{
    for (auto &e: centre_points){X.push_back(e.x); Y.push_back(e.y); V.push_back(e.velocity);}
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
			centre_points[1].velocity = std::min(f_gain * (sqrt(centre_points[0].radius) 
									+ sqrt(centre_points[1].radius)) / 2, v_max);

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

void PathPlanner::addCentrePoints(const float &car_x, const float &car_y)
{
	int opp_idx;
	float midpoint_x;
	float midpoint_y;

	// Check whether cones are mapped already
	for (size_t i = l_cone_index; i < left_cones.size(); i++)
	{
		// Left cones
		if (!left_cones[i]->mapped)
		{
			opp_idx = findOppositeClosest(*left_cones[i], right_cones);
			if (opp_idx != -1) // if solution found
			{
				midpoint_x = (right_cones[opp_idx]->position.x + left_cones[i]->position.x) / 2;	
				midpoint_y = (right_cones[opp_idx]->position.y + left_cones[i]->position.y) / 2;	
				left_cones[i]->mapped = true;
				l_cone_index++;

				// Check distance to previous centre point
				PathPoint midpoint(midpoint_x, midpoint_y);
				float dist_cp = calcDist(midpoint, centre_points.back());

				// Check distance to car
				PathPoint car_pos(car_x, car_y);
				float dist_car = calcDist(car_pos, midpoint);

				if (centre_points.back().x != midpoint_x && 
					centre_points.back().y != midpoint_y && 
					dist_cp < 10 &&
					dist_car > 1)
				{
					centre_points.push_back(midpoint);
				}
			}
		}
	}
}

void PathPlanner::addCones(const std::vector<Cone> &new_cones)
{
	size_t stored_cone_size = left_cones.size() + right_cones.size();
	std::cout << raw_cones.size() << std::endl;

	if (stored_cone_size < new_cones.size())
	{
		for (size_t i = 0; i < new_cones.size(); i++)
		{
			raw_cones.push_back(new_cones[i]);
			if (new_cones[i].colour == 'b')
			{
				l_cones_to_add.push_back(&raw_cones.back());
				l_cones_sorted = false;
			}
			else if (new_cones[i].colour == 'y')
			{
				r_cones_to_add.push_back(&raw_cones.back());
				r_cones_sorted = false;
			}
			else if (new_cones[i].colour == 'r')
			{
				timing_cones.push_back(&raw_cones.back());
			}
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
			    l_cone_index++;
			    r_cone_index++;
		    }
	    }
    }
}

int PathPlanner::findOppositeClosest(const Cone &cone, const std::vector<Cone*> &cones)
{
	float min_dist = 5.5;
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
		std::cout << dist << std::endl;
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
		if (dist < 12)
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

     
