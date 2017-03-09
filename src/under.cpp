#include <iostream>
#include <iomanip>

#include <wiringPi.h>

using namespace std;

int main ( void )
{
    int nReads, count, pin, i;
    double duty, distance;

    pin = 0;
    nReads = 2790000;

    wiringPiSetup();
    pinMode( pin, INPUT );

    for (;;)
    {
        count = 0;
        for ( i = 0; i < nReads; i++ )
        {
            count += digitalRead( pin );
        }

        duty = static_cast< double >( count ) / ( 0.75 * nReads );
        distance = 0.0254 * ( ( duty * 37500 ) / 147 );

        cout << setprecision( 6 ) << fixed;
        cout << "Distance: " << distance << " [ m ]" << endl;
    }

    return 0;
}
