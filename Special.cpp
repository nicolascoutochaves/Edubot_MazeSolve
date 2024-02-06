/* Essa e uma versao especial e muito mais complexa, que pretende no futuro levar em conta o caminho ja percorrido pelo robo.
Porem sera um esforco muito grande, visto que o espaco no labirinto e continuo, e nao discreto como o algoritmo tenta abstrair.
Essa solucao nao funciona ainda, precisa ser refinada e remover alguns bugs, mas tem grande potencial.

Por hora, ficaremos com a versao mais simples em Main.cpp, apesar de nao resolver qualquer labirinto possivel, pelo trabalho proposto e dado que e uma cadeira de 2 creditos do primeiro semestre, acho que e uma implementacao bem justa */

#include <iostream>
#include <cmath>
#include <vector>
#include "libs/EdubotLib.hpp"

#define Speed 0.1  // Robot moving speed [-1, 1]
#define Delay 1000  // Edubot delay (adjust this to obtain adequate responding behaviors)
#define MinDistance 0.07
#define Size 0.18  // Edubot size
#define MinDisplacement 0.5
#define OpenHallDistance 2

using namespace std;

// Enumerates each sonar based on Edubotlib sonar vector
enum Sonars : int { L90, L60, L30, Front, R30, R60, R90 };

// Compass directions:
enum Directions : int { E, NE, N, NW, W, SW, S, SE, Left30 = -30, Right30 = 30, Left60 = -60, Right60 = 60, Left = -90, Right = 90 };

// Define a class to represent a single cell in the grid
class GridCell {
public:
    double x, y;
    int weight;
    bool active;

    GridCell(double x, double y, int weight, bool active) : x(x), y(y), weight(weight), active(active) {}
};

// Define a class to represent the dynamic grid
class PathGrid {
private:
    int width, height;
    std::vector<std::vector<GridCell>> grid;

public:
    PathGrid(int width, int height) : width(width), height(height) {
        grid.resize(width, std::vector<GridCell>(height, GridCell(0.0, 0.0, 0, false)));
    }

    // Function to update a cell in the grid
    void updateCell(double robotX, double robotY, int weight, bool active) {
        int gridX = static_cast<int>(robotX / Size);
        int gridY = static_cast<int>(robotY / Size);

        if (gridX >= 0 && gridX < width && gridY >= 0 && gridY < height) {
            grid[gridX][gridY] = GridCell(robotX, robotY, weight, active);
        }
    }

    // Function to get the weight of a cell
    int getCellWeight(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return grid[x][y].weight;
        }
        return 0;  // Return 0 for cells outside the grid
    }

    // Function to check if a cell is active
    bool getCellActive(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return grid[x][y].active;
        }
        return false;  // Return false for cells outside the grid
    }

    // Function to print the grid (for debugging purposes)
    void printGrid() const {
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                std::cout << "(" << grid[j][i].x << ", " << grid[j][i].y << ", " << grid[j][i].weight << ", " << grid[j][i].active << ") ";
            }
            std::cout << std::endl;
        }
    }

    // Function to get the width of the grid
    int getWidth() const {
        return width;
    }

    // Function to get the height of the grid
    int getHeight() const {
        return height;
    }
};

class MazeSolver {
private:
    EdubotLib* edubotLib;
    bool canRotate;
    bool isMoving;
    float speed;
    double x, y;
    double prevX, prevY;  // Previous x and y to calculate the total displacement
    double xBuffer, yBuffer;  // Previous x and y to calculate the displacement since the robot identifies an Open Path
    double displacement;  // Stores the total x and y displacement since the last turn;
    PathGrid pathGrid;  // Dynamic grid to store the robot's path
    int lastGridX, lastGridY;

public:
    MazeSolver(EdubotLib* lib, int gridWidth, int gridHeight)
        : edubotLib(lib), canRotate(false), isMoving(false), speed(Speed), x(0.0), y(0.0), pathGrid(gridWidth, gridHeight) {}

    // Function to get the weight of the current cell in the grid
    int getCurrentCellWeight() const {
        int gridX = static_cast<int>(x / Size);
        int gridY = static_cast<int>(y / Size);
        return pathGrid.getCellWeight(gridX, gridY);
    }

    // ... (rest of the MazeSolver class remains unchanged)

    void tryOpenPath() {
        // Verifies if there is an open path in left or side while is moving towards
        if (!canRotate && (edubotLib->getSonar(L90) >= OpenHallDistance || edubotLib->getSonar(R90) >= OpenHallDistance) && displacement > MinDisplacement) {
            xBuffer = x;  // Saves the current x and y location
            yBuffer = y;
            canRotate = true;  // Change to true for not entry in this "if" until the robot turns (in the next "if")

            cout << "Found an open path!" << endl;
        }

        if (canRotate && (fabs(x - xBuffer) > Size || fabs(y - yBuffer) > Size) && displacement > MinDisplacement) {
            xBuffer = x;
            yBuffer = y;
            isMoving = false;

            // Update the cell in the grid where the robot turned
            int gridX = static_cast<int>(x / Size);
            int gridY = static_cast<int>(y / Size);
            pathGrid.updateCell(x, y, 1, true);

            if ((edubotLib->getSonar(L90) >= OpenHallDistance || edubotLib->getSonar(R90) >= OpenHallDistance)) {
                // Confirms if there is an open path
                edubotLib->stop();
                edubotLib->sleepMilliseconds(Delay);  // Delay to wait for the real robot actually executes the action

                cout << "Turning to the side open path" << endl;

                int currentCellWeight = getCurrentCellWeight();
                if (currentCellWeight < 5) {
                    // Rotate to the side with lower weight
                    if (pathGrid.getCellWeight(gridX - 1, gridY) < pathGrid.getCellWeight(gridX + 1, gridY)) {
                        edubotLib->rotate(Left);
                    }
                    else {
                        edubotLib->rotate(Right);
                    }
                }
                else {
                    // Default behavior when the cell weight is high
                    if (edubotLib->getSonar(L90) > edubotLib->getSonar(R90)) {
                        edubotLib->rotate(Left);
                    }
                    else if (edubotLib->getSonar(L90) <= edubotLib->getSonar(R90)) {
                        edubotLib->rotate(Right);
                    }
                }

                edubotLib->sleepMilliseconds(Delay);
                displacement = 0;
                prevX = x;
                prevY = y;
            }

            canRotate = false;  // Allow the robot to verify again if there is an open space on its sides
        }
    }


    double fixAngle(double current) {
        double compass[] = { 0.0, 90.0, 180.0, 270.0 };
        double smallest = 100000.0;
        int smallestIndex = -1;

        if (current == 360)
            current = 0;

        for (int i = 0; i < 4; i++) {
            if (fabs(current - compass[i]) < smallest) {
                smallest = fabs(current - compass[i]);
                smallestIndex = i;
            }
        }

        return compass[smallestIndex];
    }

    double convertAngle(double target) {
        double angle;
        double current = edubotLib->getTheta();

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

        if (angle == 180)
            angle = 0;

        return angle;
    }
    void avoidWalls() {
        // Test if the robot is too close to a wall
        if (edubotLib->getSonar(Front) < MinDistance || edubotLib->getSonar(L30) < MinDistance || edubotLib->getSonar(R30) < MinDistance) {
            if (isMoving) {
                edubotLib->stop();
                isMoving = false;
                edubotLib->sleepMilliseconds(Delay);

                double theta = edubotLib->getTheta();
                // Fix the angle to continue moving straight
                if (fmod(theta, 90.0) != 0.0) {
                    edubotLib->rotate(convertAngle(fixAngle(theta)));
                    edubotLib->sleepMilliseconds(Delay);
                }

                // Compare the sonars to rotate to the greatest distance
                if (edubotLib->getSonar(L90) > edubotLib->getSonar(R90)) {
                    edubotLib->rotate(Left);
                    edubotLib->sleepMilliseconds(Delay);
                }
                else if (edubotLib->getSonar(L90) <= edubotLib->getSonar(R90)) {
                    edubotLib->rotate(Right);
                    edubotLib->sleepMilliseconds(Delay);
                }

                displacement = 0;  // Store the total displacement since the last turn
                prevX = x;
                prevY = y;
            }
        }
    }

    void move() {
        // Moves the robot
        if (!isMoving) {
            edubotLib->sleepMilliseconds(Delay);
            edubotLib->move(speed);
            isMoving = true;

            if (speed < 0) {
                edubotLib->sleepMilliseconds(300);
                edubotLib->stop();
            }
        }
    }
    void solveMaze() {
        while (true) {

            // Update the cell in the grid where the robot is currently
            int gridX = static_cast<int>(x / Size);
            int gridY = static_cast<int>(y / Size);
            if (gridX != lastGridX || gridY != lastGridY) {
                pathGrid.updateCell(x, y, 1, true);
                lastGridX = gridX;
                lastGridY = gridY;
            }
            speed = Speed;
            x = edubotLib->getX();
            y = edubotLib->getY();
            displacement = sqrt(pow(x - prevX, 2) + pow(y - prevY, 2));

            tryOpenPath();
            avoidWalls();
            move();

            for (int i = 0; i < 2; i++) {
                if (edubotLib->getBumper(i)) {
                    speed = (i < 2) ? -0.1 : 0.1;
                    isMoving = false;
                }
            }

            double wallsTotalDistance = 0.0;
            for (int i = 0; i < 7; i++) {
                wallsTotalDistance += edubotLib->getSonar(i);
            }

            if (wallsTotalDistance >= (9)) {
                edubotLib->sleepMilliseconds(Delay * 2);
                edubotLib->stop();
                break;
            }
        }
    }

};

int main() {
    EdubotLib* edubotLib = new EdubotLib();

    if (edubotLib->connect()) {
        int showSensorsTimes = 10;

        while (showSensorsTimes > 0) {
            edubotLib->sleepMilliseconds(200);

            for (int i = 0; i < 7; i++) {
                cout << "S" << i << ": " << edubotLib->getSonar(i) << "m, ";
            }

            for (int i = 0; i < 4; i++) {
                cout << "B" << i << ": " << (edubotLib->getBumper(i) ? "true" : "false") << ", ";
            }

            cout << "leftCount: " << edubotLib->getEncoderCountLeft() << ", ";
            cout << "rightCount: " << edubotLib->getEncoderCountRight() << ", ";
            cout << "dt(loopTime): " << edubotLib->getEncoderCountDT() << ", ";
            cout << "x: " << edubotLib->getX() << ", ";
            cout << "y: " << edubotLib->getY() << ", ";
            cout << "theta: " << edubotLib->getTheta() << ", ";
            cout << "battCell0: " << edubotLib->getBatteryCellVoltage(0) << ", ";
            cout << "battCell1: " << edubotLib->getBatteryCellVoltage(1) << ", ";
            cout << "battCell2: " << edubotLib->getBatteryCellVoltage(2) << endl;

            showSensorsTimes--;
        }

        MazeSolver bot(edubotLib, 30, 30);
        bot.solveMaze();
        cout << "Maze is solved!" << endl;
        edubotLib->disconnect();
    }
    else {
        cout << "Could not connect on robot!" << endl;
    }

    delete edubotLib;
    return 0;
}

