#include "programDeclarations.h"
/*
win_64_sim.c
esp32-s3.c
*/
#include "win_64_sim.c"
// #include "esp32-s3.c"

int main(int argc, char const *argv[])
{
    setup();

    while (persistFlag)
    {
        preRoutine();
        physicalInputRoutine();
        sensorRoutine();
        motorRoutine();
    }
    return 0;
}
