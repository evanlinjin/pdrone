#ifndef GPIO_H_
#define GPIO_H_
#include<string>
#include<fstream>
using std::string;
using std::ofstream;

#define GPIO_PATH "/sys/class/gpio/"

typedef int (*CallbackType)(int);

/**
 * @class GPIO
 * @brief GPIO class for input and output functionality on a single GPIO pin
 */
class GPIO {
public:
	enum DIRECTION{ INPUT, OUTPUT };
	enum VALUE{ LOW=0, HIGH=1 };
	enum EDGE{ NONE, RISING, FALLING, BOTH };

private:
	int number;			/**< The GPIO number of the object */
	int debounceTime;   /**< The debounce time in milliseconds */
	string name;		/**< The name of the GPIO e.g. gpio50 */
	string path;  		/**< The full path to the GPIO e.g. /sys/class/gpio/gpio50/ */

public:
	GPIO(int number);
	virtual int getNumber() { return number; } /**< Returns the GPIO number as an int. */

	// General Input and Output Settings
	virtual int setDirection(GPIO::DIRECTION);
	virtual GPIO::DIRECTION getDirection();
	virtual int setValue(GPIO::VALUE);
	virtual int toggleOutput();
	virtual GPIO::VALUE getValue();
	virtual int setActiveLow(bool isLow=true);  //low=1, high=0
	virtual int setActiveHigh(); //default
	//software debounce input (ms) - default 0
	virtual void setDebounceTime(int time) { this->debounceTime = time; }

	// Advanced OUTPUT: Faster write by keeping the stream alive (~20X)
	virtual int streamOpen();
	virtual int streamWrite(GPIO::VALUE);
	virtual int streamClose();

	virtual int toggleOutput(int time); //threaded invert output every X ms.
	virtual int toggleOutput(int numberOfTimes, int time);
	virtual void changeToggleTime(int time) { this->togglePeriod = time; }
	virtual void toggleCancel() { this->threadRunning = false; }

	// Advanced INPUT: Detect input edges; threaded and non-threaded
	virtual int setEdgeType(GPIO::EDGE);
	virtual GPIO::EDGE getEdgeType();
	virtual int waitForEdge(); // waits until button is pressed
	virtual int waitForEdge(CallbackType callback); // threaded with callback
	virtual void waitForEdgeCancel() { this->threadRunning = false; }

	virtual ~GPIO();  //destructor will unexport the pin

private:
	int exportGPIO();
	int unexportGPIO();
	ofstream stream;
	pthread_t thread;
	CallbackType callbackFunction;
	bool threadRunning;
	int togglePeriod;  //default 100ms
	int toggleNumber;  //default -1 (infinite)
	friend void* threadedPoll(void *value);
	friend void* threadedToggle(void *value);
};

void* threadedPoll(void *value);
void* threadedToggle(void *value);

#endif /* GPIO_H_ */