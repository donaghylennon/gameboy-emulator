#include "CPU.h"

int main(int argc, char **argv) {
    CPU test(argv[1]);
    test.run();
    return 0;
}
