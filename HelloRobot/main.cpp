#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cmath>

using namespace std;
using namespace PlayerCc;

#define SAMPLES 666 // Number of Samples 240/0.36 from wbr914sim.cfg
#define MIN_DISTANCE_TO_OBSTACLE 0.7 // Minimum distance to rotate.
#define CHECK_RADIUS 65 // Maximum radius to check.
#define MIN_ANGLE_FOR_AVOIDANCE -40
#define MAX_ANGLE_FOR_AVOIDANCE 40

int deg_to_index(double deg)
{
	return (deg + 120) / 0.36;
}

double index_to_deg(int index)
{
	return ((-120+index*0.36)* M_PI ) / 180;
}

int main(int argc, char** argv)
{
	PlayerClient pc("localhost",6665);
	Position2dProxy pp(&pc);
	LaserProxy lp(&pc);
	cout << "starting";

	pp.SetMotorEnable(true); // for the real robot only.
	pp.SetOdometry(-0.99,0.04,45.1);
	while (true){
		pc.Read();

		bool canMoveForward = true;
		int minIndex = deg_to_index(MIN_ANGLE_FOR_AVOIDANCE);
		int maxIndex = deg_to_index(MAX_ANGLE_FOR_AVOIDANCE);
		int countObstacle = 0;

		for (int i = minIndex; i<maxIndex; i++)
		{
			if (lp[i] < MIN_DISTANCE_TO_OBSTACLE)
			{
					canMoveForward = false;
					countObstacle++;
			}
		}

		if (canMoveForward)
		{
			pp.SetSpeed(0.5, 0);
		}
		else if (countObstacle >= 0.6 * (maxIndex - minIndex))
		{
			pp.SetSpeed(0, M_PI);
		}
		else
		{

			double xRobot = pp.GetXPos();
			double yRobot = pp.GetYPos();

			for(int i = 0; i < SAMPLES ; i++){

				double xObs = xRobot + lp[i] * cos(pp.GetYaw() + index_to_deg(i));
				double yObs = yRobot + lp[i] * sin(pp.GetYaw() + index_to_deg(i));

				if (lp[i] < MIN_DISTANCE_TO_OBSTACLE){
					cout << "(" << xObs << ",";
					cout << "" << yObs << ")\n";
				}

			}

			int current = SAMPLES / 2;
			int last_known_left_obstacle = -1;
			int last_known_right_obstacle = -1;
			int minIndex = deg_to_index(MIN_ANGLE_FOR_AVOIDANCE) * 2;
			int maxIndex = deg_to_index(MAX_ANGLE_FOR_AVOIDANCE) * 2;

			for (int i = 0; i < CHECK_RADIUS; i++)
			{
				// Check obstacles to the left
				if (lp[current + i] < MIN_DISTANCE_TO_OBSTACLE)	{
					last_known_left_obstacle = i;
				}

				// Check obstacles to the right
				if (lp[current - i] < MIN_DISTANCE_TO_OBSTACLE)	{
					last_known_right_obstacle = i;
				}

				//printf("(%d, %d) \n",last_known_left_obstacle, last_known_right_obstacle);
				if (last_known_left_obstacle == -1 && last_known_right_obstacle == -1)
				{
					pp.SetSpeed(0.5, 0);
				}
				else if ((last_known_left_obstacle == -1) || (last_known_left_obstacle < last_known_right_obstacle))
					pp.SetSpeed(0, 0.25 * M_PI);
				else if ((last_known_right_obstacle == -1) || (last_known_left_obstacle > last_known_right_obstacle))
				{
					pp.SetSpeed(0, -0.25 * M_PI);
				}
				else
				{
					pp.SetSpeed(0, -0.5 * M_PI);
				}
			}
		}
	}
}

