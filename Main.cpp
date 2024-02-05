#include <iostream>
#include "libs/EdubotLib.hpp"
#include <cmath>

#define SPEED 0.1 //Robot moving speed [-1, 1]
#define DELAY 1000 //Edubot delay (adjust this to obtain adequate responsing behaviours)
#define MIN_DISTANCE 0.06
#define SIZE 0.25 //Edubot size
using namespace std;

//Enumerates each sonar based in Edubotlib sonar vector
enum SONARS : int { L_90, L_60, L_30, FRONT, R_30, R_60, R_90 };

//Compass directions:
enum DIRECTIONS : int { E, NE, N, NW, W, SW, S, SE, LEFT_30 = -30, RIGHT_30 = 30, LEFT_60 = -60, RIGHT_60 = 60, L = -90, R = 90, };

typedef struct PATH {
	double x, y;
	int weight;
	bool active;
}PATH;

//Edubot main class used to interact with the real hardware and the simulator
EdubotLib* edubotLib = new EdubotLib();
//PATH paths[1000] = {0};

//Returns the real angle based in the current bot theta and the target angle
double Fix_Angle(double current) {
    double compass[] = { 0.0, 90.0, 180.0, 270.0};
    double menor = 100000.0;
    int indiceMenor = -1;
    
    if(current == 360)
        current = 0;

    for (int i = 0; i < 4; i++) {
        if (fabs(current - compass[i]) < menor) {
            menor = fabs(current - compass[i]);
            indiceMenor = i;
        }
    }

    return compass[indiceMenor];
}
double angle(double target) {
	double angle;
	double current = edubotLib->getTheta();
	//cout << "trying to rotate to angle: " << target << endl;

	if (current <= 180) {
		if (target <= 180) {
			angle = current - target;
		}
		else {
			angle = current - target + 360;
		}
	}
	else {
		if (target > 180) {
			angle = current - target;
		}
		else {
			angle = current - target - 360;
		}
	}
	if(angle == 180)
		angle = 0;
	//cout << "normalized angle = " << angle << endl << endl;
	return angle;

}

int main() {


	//try to connect on robot
	if (edubotLib->connect()) {

		int showSensorsTimes = 10; // shows edubot sensors
		float speed = SPEED;
		double x, y, prev_x = 0, prev_y = 0;
		bool CanRotate = false;
		bool IsMoving = false;
		//bool IsCrashed = false;



		while (showSensorsTimes > 0) {

			// Waits for two seconds
			edubotLib->sleepMilliseconds(200);

			// sonars
			for (int i = 0; i < 7; i++) {
				std::cout << "S" << i << ": " << edubotLib->getSonar(i) << "m, ";
			}

			// bumpers
			for (int i = 0; i < 4; i++) {
				std::cout << "B" << i << ": " << (edubotLib->getBumper(i) ? "true" : "false") << ", ";
			}

			std::cout << "leftcount: " << edubotLib->getEncoderCountLeft() << ", ";
			std::cout << "rightcount: " << edubotLib->getEncoderCountRight() << ", ";
			std::cout << "dt(looptime): " << edubotLib->getEncoderCountDT() << ", ";

			std::cout << "x: " << edubotLib->getX() << ", ";
			std::cout << "y: " << edubotLib->getY() << ", ";
			std::cout << "theta: " << edubotLib->getTheta() << ", ";

			std::cout << "battCell0: " << edubotLib->getBatteryCellVoltage(0) << ", ";
			std::cout << "battCell1: " << edubotLib->getBatteryCellVoltage(1) << ", ";
			std::cout << "battCell2: " << edubotLib->getBatteryCellVoltage(2);

			// line break
			std::cout << std::endl;

			showSensorsTimes--;

		}

		edubotLib->rotate(angle(0));
		edubotLib->sleepMilliseconds(DELAY);


		while (true) {
			speed = SPEED; //SPEED = 0.1
			//gets the X and Y positions calculated by the robot
			x = edubotLib->getX();
			y = edubotLib->getY();


			//Verifies if there is an open path in left or side while is moving towards

			if(!CanRotate && (edubotLib->getSonar(L_90) >= 2.0 || edubotLib->getSonar(R_90) >= 2.0)){
					prev_x = x; //saves the current x and y location
					prev_y = y;
					CanRotate = true; //change to true for not entry in this "if" until the robot turns (in the next "if")
			}
			if(CanRotate && (fabs(x - prev_x) > SIZE || fabs(y - prev_y) > SIZE)){ //Starts rotating only after the robot walk through your own SIZE
				prev_x = x;
				prev_y = y;

				edubotLib->stop();
				IsMoving = false;
				edubotLib->sleepMilliseconds(DELAY); //Delay to wait the real robot actually executes the action

				if (edubotLib->getSonar(L_90) > edubotLib->getSonar(R_90)) {
					edubotLib->rotate(L);
					edubotLib->sleepMilliseconds(DELAY);
				}
				else if (edubotLib->getSonar(L_90) <= edubotLib->getSonar(R_90)) {
					edubotLib->rotate(R);
					edubotLib->sleepMilliseconds(DELAY);
				}
				CanRotate = false; //allow the robot to verify again if there is an open space in its sides
			}

			//Test is the robot is too close to a wall
			if (edubotLib->getSonar(FRONT) < MIN_DISTANCE || edubotLib->getSonar(L_30) < MIN_DISTANCE || edubotLib->getSonar(R_30) < MIN_DISTANCE) {
				
				if (IsMoving) {
					edubotLib->stop();
					IsMoving = false;
					edubotLib->sleepMilliseconds(DELAY);

					double theta = edubotLib->getTheta(); //Fix the angle to continue moving straigh
					if (fmod(theta, 90.0) != 0.0) {
						edubotLib->rotate(angle(Fix_Angle(theta))); //angle() returns an angle beetwen [-180, 180] comparing the current angle to the target angle and returning the delta necessary to reach the targe angle and Fix_Angle returns the closest angle the robot are as the target angle for the function angle().
						edubotLib->sleepMilliseconds(DELAY);
					}

				}

				//Compare the sonars to rotate to the greatest distance
				if (edubotLib->getSonar(L_90) > edubotLib->getSonar(R_90)) {
					edubotLib->rotate(L); //L = -90
					edubotLib->sleepMilliseconds(DELAY);
				}
				else if (edubotLib->getSonar(L_90) <= edubotLib->getSonar(R_90)){
					edubotLib->rotate(R); //R = +90
					edubotLib->sleepMilliseconds(DELAY);
				}
				
				//Compare all the sensors to verify if the robot is in outdoors
				float walls_total_distance = 0;
				for (int i = 0; i < 7; i++) {
					walls_total_distance += edubotLib->getSonar(i);
				}
				if (walls_total_distance >= (float)(7.5)) {				
					cout << "Maze is solved!" << endl;
					break; //breaks the main loop
				}


			}

			//Verify the four bumpers to deal with wall colisions
			for(int i = 0; i < 2; i++){
				if(edubotLib->getBumper(i) ){
					speed = (i < 2)? -0.1: 0.1;
					IsMoving = false;
				}
			}
			
			//Moves the robot
			if (!IsMoving) {
				edubotLib->sleepMilliseconds(DELAY);
				edubotLib->move(speed);
				IsMoving = true;
				if(speed < 0){
					edubotLib->sleepMilliseconds(300);
					edubotLib->stop();
				}
			} 




		}
		edubotLib->disconnect();
	}
	else {
		std::cout << "Could not connect on robot!" << std::endl;
	}

	return 0;
}
