#include "CPU.h"

#include <iostream>

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Error: no ROM provided\n");
        exit(1);
    }
    CPU test(argv[1]);
    test.run();
    return 0;
}
