#include <iostream>
#include <cmath>
using namespace std;

double Fix_Angle(double current) {
    double compass[4] = { 0.0, 90.0, 180.0, 270.0 };
    double menor = 100000.0;
    int indiceMenor = -1;

    cout << endl << "Inside fix_angle:" << endl << "current theta: " << current << endl;
    
    if(current == 360)
        current = 0;

    for (int i = 0; i < 4; i++) {
        cout << endl << " " << current << " - " << compass[i]  << " = " << fabs(current - compass[i]) << endl;
        if (fabs(current - compass[i]) < menor) {
            menor = fabs(current - compass[i]);
            indiceMenor = i;
        }
    }

    cout << " menor: " << compass[indiceMenor] << endl << endl;
    return compass[indiceMenor];
}

int main(){
    Fix_Angle(0.7);
    Fix_Angle(98.65);
    Fix_Angle(173.8);
    Fix_Angle(297.77);

    return 0;
}
