#ifndef worhp_info
#define worhp_info

#include "../base/defines.h"
#include <iostream>
#include "worhp/worhp.h"

std::ostream &operator<<(std::ostream& os, const OptVar &o);

void print(WorhpMatrix &M, int index, std::ostream &os = std::cout);


#endif

