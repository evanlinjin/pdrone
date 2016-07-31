#include "GPIO_OUT.h"
#include "util.h"

GPIO_OUT::GPIO_OUT(uint pin) {
    this->pin = pin;
    this->path = (string)GPIO_PATH + (string)"gpio" + to_string(pin) + (string)"/";

    // SETUP >>
	write(GPIO_PATH, "export", this->pin);
    while (setup() == -1) {usleep(50000);}

    // INITIATE 'ofstream' OBJECT >>
    value.open((this->path + "value").c_str());
}

void GPIO_OUT::set_value(uint value) {
    this->value << value << std::flush;
}

GPIO_OUT::~GPIO_OUT() {
    write(this->path, "value", 0);
    value.close();
    //write(GPIO_PATH, "unexport", this->pin);
}

int GPIO_OUT::setup() {
    int x = write(this->path, "direction", "out");
    write(this->path, "value", 0);
    return x;
}