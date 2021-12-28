#ifndef __DEV_H__
#define __DEV_H__
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <deque>
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
    std::mutex _mtx;
    std::thread* _thread;
    deque<unsigned char> recv_buff;     // 接收缓存
    vector<vector<unsigned char>> frame; // 解析成功的帧

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
private:
    void recv_thread();
};

#endif
