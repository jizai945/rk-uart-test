#ifndef __DEV_H__
#define __DEV_H__
#include <iostream>
#include <string>
using namespace std;


enum DEV_STATE {
    DEV_CLOSE = 1,
    DEV_OPEN = 2
};

class DEV {
private:
    string dev_name;
    int bound;
    int fd;
    DEV_STATE state;

public:
    DEV() = delete;
    DEV(string dev, int bound);
    ~DEV();
    bool popen();
    bool pclose();
    bool set_channel(unsigned char ch);
    bool start_send_test(int count);
    bool start_recv_test(int time);
    int get_version(void);
};

#endif
