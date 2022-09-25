#include "CPU.h"

#include <iostream>

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Error: no ROM provided\n");
        exit(1);
    }
    bool log = false;
    if (argc >= 3 && !strcmp("--log", argv[2]))
        log = true;
    CPU test(argv[1], log);
    test.run();
    return 0;
}
