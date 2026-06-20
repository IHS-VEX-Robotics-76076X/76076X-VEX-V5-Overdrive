#include <iostream>
#include <cstdlib>

#include "main.h"
#include "util.hpp"

// js playing arnd w/ cpp u can remove this vvv
void fun() {
    int random1 = (std::rand() % 100) + 1;
    int random2 = (std::rand() % random1) + 1;
    int random3 = (std::rand() % random2) + 1;
    
    std::cout << "roll is " << random3 << "\n";
    if (random3 == 100) std::cout << "wow 1 in a million you rolled 100" << std::endl;
}