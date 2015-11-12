#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace std;
using namespace PlayerCc;

#define SAMPLES 666 // Number of Samples 240/0.36 from wbr914sim.cfg
#define MIN_DISTANCE_TO_OBSTACLE 0.9 // Minimum distance to rotate.
#define CHECK_RADIUS 50 // Maximum radius to check.


int main(int argc, char** argv)
{
	PlayerClient pc("localhost",6665);
	Position2dProxy pp(&pc);
	LaserProxy lp(&pc);
	cout << "starting";

	pp.SetMotorEnable(true); // for the real robot only.

	while (true){

		pc.Read();

		int current = SAMPLES / 2;
		int last_known_left_obstacle = -1;
		int last_known_right_obstacle = -1;

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
		}

		// No obstacles. Go forward.
		if (last_known_left_obstacle == -1 && last_known_right_obstacle == -1)
			pp.SetSpeed(0.2, 0);
		else{
			if (last_known_left_obstacle > last_known_right_obstacle){
				pp.SetSpeed(0, -0.5);
			}
			else{
				pp.SetSpeed(0, 0.5);
			}
		}
	}
}
