#include "GPIO_IN.h"
#include "util.h"

GPIO_IN::GPIO_IN(uint pin, string edge_type, uint active_low) {
    this->pin = pin;
    this->path = (string)GPIO_PATH + (string)"gpio" + to_string(pin) + (string)"/";

    // SETUP >>
	write(GPIO_PATH, "export", this->pin);
    usleep(50000);
    write(this->path, "direction", "in");
    write(this->path, "edge", edge_type);
    write(this->path, "active_low", active_low);

    // INITIATE 'ifstream' OBJECT >>
    value.open((this->path + "value").c_str());
}

GPIO_IN::~GPIO_IN() {
    value.close();
    //write(this->path, "value", 0);
    //write(GPIO_PATH, "unexport", this->pin);
    if (this->using_epoll) {close(fd);}
}

uint GPIO_IN::get_value() {
    value.seekg(0);
    return (value.get() - '0');
}

void GPIO_IN::epoll_setup() {
    this->using_epoll = true;
    this->epollfd = epoll_create(1);
    this->fd = open((this->path + "value").c_str(), O_RDONLY | O_NONBLOCK);

    // Error Checking >>
    if (epollfd == -1) {perror("GPIO: Failed to create epollfd"); return;}
    if (fd == -1) {perror("GPIO: Failed to open file"); return;}

    // ev.events = read operation | edge triggered | urgent data >>
    this->ev.events = EPOLLIN | EPOLLET | EPOLLPRI;
    this->ev.data.fd = fd;

    if (epoll_ctl(this->epollfd, EPOLL_CTL_ADD, this->fd, &this->ev) == -1 ) {
        perror("GPIO: Failed to add control interface");
    }
    return;
}

bool GPIO_IN::epoll_wait_once() {
    return epoll_wait(epollfd, &ev, 1, 5000);
}