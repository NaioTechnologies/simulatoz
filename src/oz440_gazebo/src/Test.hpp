//
// Created by fanny on 25/11/16.
//

#ifndef PROJECT_TEST_H
#define PROJECT_TEST_H

#include <iostream>
#include <mutex>
#include <string>
//#include "std_msgs/String.h"

class Test {

public:
    // Constructeur/destructeur
    Test( );
    ~Test( );

    std::string get_type_vegetable();
    int get_number_rows();
    int get_length_rows();
    float get_width();

private:
    //functions

private:

    //Test parameters
    std::string type_vegetable_;
    int number_rows_;
    int length_rows_;
    float width_;

};


#endif //PROJECT_TEST_H
