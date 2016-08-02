#include "PWM.h"
#include "util.h"

PWM::PWM(uint n_chip, uint period) {
	cout << "Initialising pwmchip" << n_chip << " ..." << endl;

	this->path = PWM_PATH + to_string(n_chip) + "/";
	this->period = period;

	cout << "PATH: " << this->path << endl;

	// ENABLE PINS >>
	short is_okay[2] = {-1, -1};
	while (is_okay[0] == -1 || is_okay[1] == -1) {
		is_okay[0] = write(this->path, "export", 0);
		is_okay[1] = write(this->path, "export", 1);
		usleep(50000);
	}

	// PERMISSIONS :
	system("sudo chown -R root:gpio /sys/class/pwm");
	system("sudo chmod -R 777 /sys/class/pwm");
	system("sudo chown -R root:gpio /sys/devices/platform/ocp");
	system("sudo chmod -R 777 /sys/devices/platform/ocp");

	// SETUP >>
	while (setup() == -1) {usleep(50000);}

	// INITIATE 'ofstream' OBJECTS >>
	// duty_pwm[0] = new ofstream;
	// duty_pwm[1] = new ofstream;
	// duty_pwm[0]->open((this->path + "pwm0/duty_cycle").c_str());
	// duty_pwm[1]->open((this->path + "pwm1/duty_cycle").c_str());
}

PWM::~PWM() {
	write(this->path, "pwm0/duty_cycle", 0);
	write(this->path, "pwm1/duty_cycle", 0);

	// duty_pwm[0]->close();
	// duty_pwm[1]->close();
	// delete duty_pwm[0];
	// delete duty_pwm[1];
	
	//write(this->path, "pwm0/enable", 0);
	//write(this->path, "pwm1/enable", 0);
	// write(this->path, "unexport", 0);
	// write(this->path, "unexport", 1);
}

void PWM::set_duty(uint pin, uint duty) {
	//*duty_pwm[pin] << duty << std::flush;
	write(this->path, "pwm" + to_string(pin) + "/duty_cycle", duty);
	//echo(to_string(duty), "pwm" + to_string(pin) + "/duty_cycle");
}

void PWM::set_duty_pc(uint pin, float pc) {
	this->set_duty(pin, (uint)( (float)this->period * pc / 100.0 ) );
}

// void PWM::echo(string value, string file) {
// 	// string cmd = "sudo sh -c \" echo " + value + " > " + path + file + "\"";
// 	string cmd = "echo " + value + " > " + path + file;
// 	cout << "RUNNING : " << cmd << endl;
// 	system(cmd.c_str());
// }

int PWM::setup() {
	int x = write(this->path, "pwm0/period", period);
	write(this->path, "pwm1/period", period);
	write(this->path, "pwm0/duty_cycle", 0);
	write(this->path, "pwm1/duty_cycle", 0);
	write(this->path, "pwm0/enable", 1);
	write(this->path, "pwm1/enable", 1);
	return x;
}
