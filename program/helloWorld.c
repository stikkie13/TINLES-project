#include <stdio.h>
// Definitions and declarations
#define NL printf("\n")

struct motionSensorStruct
{
    int x;
    int y;
    int z;
    float noise;
};

void printMotionSensor(struct motionSensorStruct motionSensors);

// Declare (and define) arrays to point to
int ints[4] = {0, 1, 2, 3};
float floats[4] = {0, 0.1, 0.2, 0.3};
struct motionSensorStruct motionSensors[4];

// declare an array of void pointers to arrays of different types.
/*
0: ints
1: floats
2: structs*/
void *ptrs[] = {ints, floats, motionSensors};

int main(int argc, char const *argv[])
{

    printf("Hello world\n");
    NL;

    printf("\tThis program demonstates the use of an array of voidpointers.\nUsing an array of voidpointers we can declare a function as 'void func(void* inputPntrs)'.\n");
    NL;
    NL;

    // Make an int pointer at the adress in ptrs[0]
    int *intPtr = (int *)ptrs[0];
    // Make float pointer
    float *floatPtr = (float *)ptrs[1];
    // Make sensorPointer
    struct motionSensorStruct *motionSensorPtr = ptrs[2];

    printf("\tPrinting all ints and floats using the pointers\n");
    for (int i = 0; i < 4; i++)
    {
        // Points to the int array at the ptrs[0] adress.
        // printf("ints[%i]:%i  ", i, ints[i]);
        printf("intPtr[%i]:%i\t", i, intPtr[i]);
        // printf("floats[%i]:%2.2f  ", i, floats[i]);
        printf("floatPtr[%i]:%2.2f\t", i, floatPtr[i]);
        NL;
    }
    NL;

    printf("\tPrinting all ints and floats using ( (int*)ptrs[0] )[i]\n");
    for (int i = 0; i < 4; i++)
    {
        // Points to the int array at the ptrs[0] adress.
        printf("int:%i\t", ((int *)ptrs[0])[i]);
        printf("float:%2.3f\t", ((float *)ptrs[1])[i]);
        NL;
    }
    NL;
    NL;

    // Prepare structs for comparison
    int j = 0;
    for (int i = 0; i < 4; i++)
    {
        motionSensors[i].x = j++;
        motionSensors[i].y = j++;
        motionSensors[i].z = j++;
        motionSensors[i].noise = i * 0.1;
    }

    // Compare structs
    printf("\tComparing structs\n");
    printf("motionSensors  [3]: ");
    printMotionSensor(motionSensors[3]);
    printf("motionSensorPtr[3]: ");
    printMotionSensor(motionSensorPtr[3]);
    NL;

    // Use motionSensorPtr to read structs
    printf("\tUsing pointer to read all structs\n");
    for (int i = 0; i < 4; i++)
    {
        struct motionSensorStruct motionSensor = motionSensorPtr[i]; // Making a new variable prevents accidently changing the value we are pointing to.
        printf("motionSensorPtr[%i]:", i);
        printMotionSensor(motionSensor);
    }

    return 0;
}

void printMotionSensor(struct motionSensorStruct motionSensor)
{
    printf("x:%-2i y:%-2i z:%-2i noise:%2.2f\n", motionSensor.x, motionSensor.y, motionSensor.z, motionSensor.noise);
    return;
}