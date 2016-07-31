#ifndef PWM_H_
#define PWM_H_

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

#define PWM_PATH "/sys/class/pwm/pwmchip"

class PWM {
public:
	PWM(uint n_chip, uint period);
	~PWM();
	void set_duty(uint pin, uint duty);
	void set_duty_pc(uint pin, float pc);

private:
	uint period;
	string path;
	//ofstream* duty_pwm[2];
	// void echo(string value, string file);
	int setup();
};


#endif /* PWM_H_ */