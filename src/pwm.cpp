#include <iostream>
#include <iomanip>

#include <wiringPi.h>

using namespace std;

int main ( void )
{
    int pin;
    double distance;
    unsigned int start, end, duration;

    pin = 0;

    wiringPiSetup();
    pinMode( pin, INPUT );

    for (;;)
    {
        if ( digitalRead( pin ) )
        {
            start = micros();
            while ( digitalRead( pin ) );
            end = micros();

            duration = end - start;
            distance = 0.0254 * static_cast< double >( duration ) / 147;

            cout << setprecision( 6 ) << fixed;
            cout << "Distance: " << distance << " [ m ]" << endl;
        }
    }

    return 0;
}
