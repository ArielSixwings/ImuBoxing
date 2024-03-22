#include <iostream>

#include "src/MathUtil.h"

int main () {

    std::cout << "add(10,4): " << MathUtil::Add(10,4) << std::endl;

    return 0;
}
//cmake -S . -B build/
//cmake --build build/
