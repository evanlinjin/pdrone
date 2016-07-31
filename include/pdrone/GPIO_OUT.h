#ifndef GPIO_OUT_H_
#define GPIO_OUT_H_

#include <string>
#include <iostream>
#include<fstream>
#include <cstdlib>
#include <unistd.h>

using std::string;
using std::to_string;
using std::ofstream;
using std::cout;
using std::endl;

#define GPIO_PATH "/sys/class/gpio/"

class GPIO_OUT {
public:
    GPIO_OUT(uint pin);
    ~GPIO_OUT();
    void set_value(uint value);

private:
    uint pin;
    string path;
    ofstream value;
    int setup();
};

#endif /* GPIO_OUT_H_ */