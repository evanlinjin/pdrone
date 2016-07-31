#ifndef GPIO_IN_H_
#define GPIO_IN_H_

#include <string>
#include <iostream>
#include<fstream>
#include <cstdlib>
#include<fcntl.h>
#include<unistd.h>
#include<sys/epoll.h>

using std::string;
using std::to_string;
using std::ifstream;
using std::cout;
using std::endl;

#define GPIO_PATH "/sys/class/gpio/"

class GPIO_IN {
public:
    GPIO_IN(uint pin, string edge_type, uint active_low);
    ~GPIO_IN();
    uint get_value();

    // epoll :
    void epoll_setup();
    bool epoll_wait_once();

private:
    uint pin;
    string path;
    ifstream value;

    // epoll:
    bool using_epoll = false;
    int fd, epollfd;
    struct epoll_event ev;

};

#endif /* GPIO_IN_H_ */