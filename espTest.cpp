#include<iostream>
#include<head.h>

int main(int argc, char **argv) {
    int mode = -1, times = -1, bound = 57600, opt;
    string dev_name = "/dev/ttyUSB5";
    unsigned char channel = 6;

    while ( (opt = getopt(argc, argv, "p:b:c:m:t:")) != -1 ) {
        switch (opt) {
            case 'p':
                dev_name = optarg;
                cout << PINK << "dev name: " << dev_name << NONE << endl;
                break;
            case 'b':
                bound = atoi(optarg);
                cout << PINK << "bound: " << bound << NONE << endl;
                break;
            case 'c':
                channel = atoi(optarg);
                cout << PINK << "channel: " << optarg << NONE << endl;
                break;
            case 'm':
                mode = atoi(optarg);
                switch (mode) {
                    case 0:
                        cout << PINK << "mode: " << mode << " ->recv" << NONE << endl;
                        break;
                    case 1:
                        cout << PINK << "mode: " << mode << " ->send" << NONE << endl;
                        break;
                    case 2:
                        cout << PINK << "mode: " << mode << " ->get version" << NONE << endl;
                        break;
                }
                
                break;
            case 't':
                times = atoi(optarg);
                cout << PINK << "times: " << times << NONE << endl;
                break;
            default:
                printf(RED"optopt +%c\n" NONE,optopt);
        }
    }

    if (mode == 2) {

    }
    else if (mode == -1 || times == -1) {
        cout << RED << "mode or times is empty" << NONE << endl;
        exit(1);
    }


    DEV uart_obj(dev_name, bound);
    uart_obj.popen();
    uart_obj.set_channel(channel);
    sleep(1);
    switch (mode) {
        case 0:
            cout << BLUE << "start recv: " << NONE << endl;
            uart_obj.start_recv_test(times);
            break;
        case 1:
            cout << BLUE << "start send: "<< NONE << endl;
            uart_obj.start_send_test(times);
            break;
        case 2:
            cout << BLUE << "start get version: "<< NONE << endl;
            uart_obj.get_version();
            break;
    }

    sleep(1);
}
