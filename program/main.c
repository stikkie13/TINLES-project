#include "programDeclarations.h"
#include "sim.c"
// #include "esp32-s3.c"

int main(int argc, char const *argv[])
{
    initializeInterupts();

    while (persistFlag)
    {
        physicalInputRoutine();
        sensorRoutine();
        motorRoutine();
    }
    return 0;
}
