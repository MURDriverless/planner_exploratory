#include "path_planner.h"
#include "cone.h"
#include <iostream>
#include <ostream>
#include <vector>

int main() 
{
	// Cones test
    Cone c_1 = Cone(2.7609, 1.7154, 'b');
    Cone c_2 = Cone(3.1781, -1.9323, 'y');
    Cone c_3 = Cone(7.2786, 1.727, 'b');
    Cone c_4 = Cone(7.4856, -1.8135, 'y');
    Cone c_5 = Cone(12.035, 1.777, 'b');
    Cone c_6 = Cone(12.132, -1.684, 'y');
    Cone c_7 = Cone(16.539, 1.8575, 'b');
    Cone c_8 = Cone(16.791, -1.634, 'y');
    Cone c_9 = Cone(21.034, 1.9058, 'b');
    Cone c_10 = Cone(21.016, -1.5777, 'y');

	std::vector<Cone> test_cones = {c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, c_10};

    bool test_const_velocity = true;
    uint8_t test_v_max = 1;
    uint8_t test_v_const = 2;
    uint8_t test_max_f_gain = 3;
    float test_car_x = 0.0;
    float test_car_y = 0.0;

    PathPlanner test_planner(test_car_x, test_car_y, test_cones, test_const_velocity, test_v_max, test_v_const, test_max_f_gain);

    return 0;
}

