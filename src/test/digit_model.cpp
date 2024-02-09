#include <iostream>
#include "digit_rbkd/digit_model_library.hpp"

#define ASSERT_EQ(a, b) if (a != b) { std::cout << "Test failed: " << a << " != " << b << std::endl; return 1; }

int main(){

    DigitModel model;
    model.print_state();

    // test size
    ASSERT_EQ(model.num_bodies, 32);
//    ASSERT_EQ(model.num_sites, 5); // Can vary
    ASSERT_EQ(model.num_q, 37);
    ASSERT_EQ(model.num_v, 36);
    ASSERT_EQ(model.num_u, 20);


    return 0;
}