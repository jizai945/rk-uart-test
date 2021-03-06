#include "head.h"
using namespace std;

struct termios termios;

static int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
	B38400, B19200, B9600, B4800, B2400, B1200, B300, };

static int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300,
	38400,  19200,  9600, 4800, 2400, 1200,  300, };

static int _enable_raw_mode(int fd, struct termios *orig_termios)
{
	struct termios raw;

	if (!isatty(fd)) goto fatal;
	/* XXX:
	 if (!atexit_registered) {
	    atexit(linenoiseAtExit);
	    atexit_registered = 1;
	}*/
	if (tcgetattr(fd, orig_termios) == -1) goto fatal;

	raw = *orig_termios;  /* modify the original mode */
	/* input modes: no break, no CR to NL, no parity check, no strip char,
	 * no start/stop output control. */
	raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	/* output modes - disable post processing */
	/* raw.c_oflag &= ~(OPOST); */
	/* control modes - set 8 bit chars */
	raw.c_cflag |= (CS8);
	/* local modes - choing off, canonical off, no extended functions,
	 * no signal chars (^Z,^C) */
	raw.c_lflag &= ~(ECHO | ICANON | IEXTEN
			 /* | ISIG XXX: ignore Ctrl-C in future */
		);
	/* control chars - set return condition: min number of bytes and timer.
	 * We want read to return every single byte, without timeout. */
	raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 2; /* 1 byte, no timer */

	raw.c_cflag |= CLOCAL | CREAD;
	raw.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* ISIG */
	raw.c_oflag &= ~OPOST;
	raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

	/* put terminal in raw mode after flushing */
	if (tcsetattr(fd,TCSAFLUSH,&raw) < 0) goto fatal;
		return 0;

fatal:
	errno = ENOTTY;
	return -1;
}

static int _disable_raw_mode(int fd, struct termios *orig_termios)
{
	if (tcsetattr(fd, TCSAFLUSH, orig_termios) < 0) goto fatal;
		return 0;

fatal:
	errno = ENOTTY;
	return -1;
}

static void set_speed(int fd, int speed)
{
	int i;
	int status;
	struct termios opt;
	tcgetattr(fd, &opt);
	for (i= 0; i < sizeof(speed_arr) / sizeof(int); i++) {
		if (speed == name_arr[i]) {
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&opt, speed_arr[i]);
			cfsetospeed(&opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &opt);
			if (status != 0)
				perror("tcsetattr fd1");
			return;
		}
		tcflush(fd,TCIOFLUSH);
	}
}

/*
    crc16??????
*/
uint16_t parse_crc16 (uint8_t *buf, uint16_t len) {

    unsigned short crcin = 0xFFFF;  
    unsigned short cpoly = 0x1021;  
    unsigned char wchar = 0;  

    while (len--)     
    {  
        wchar = *(buf++);  
        crcin ^= (wchar << 8);  
        for(int i = 0;i < 8;i++)  
        {  
            if(crcin & 0x8000)  
                crcin = (crcin << 1) ^ cpoly;  
            else  
                crcin = crcin << 1;  
        }  
    }  

    return (crcin);  
}

/*
    checksum??????
*/
static uint8_t parse_checksum (uint8_t *buf, uint16_t len) {
    uint8_t sum = 0;
    
    for (int i=0; i<len; i++) {
        sum += buf[i];
    }

    return sum;

}


DEV::DEV(string dev, int bound):dev_name(dev), bound(bound) {
    state = DEV_CLOSE;
    _thread == nullptr;
}

DEV::~DEV() {
    this->pclose();
}

bool DEV::popen(){
    if (state != DEV_CLOSE) {
        cout << RED << "uart is open" << NONE << endl;
        return false;
    }
    // fd = open(dev_name.c_str(), O_RDWR);
    fd = open("/dev/ttyUSB5", O_RDWR);
    if (fd < 0) {
        perror("uart open");
        cout << RED << "open " << dev_name <<" fail" << NONE << endl;
        state = DEV_CLOSE;
        return false;
    }

    state = DEV_OPEN;
    set_speed(fd, bound);
    _enable_raw_mode(fd, &termios);

    cout << BLUE << "open " << dev_name << " bound " << bound <<" sucess" << NONE << endl;
    return true;
}

bool DEV::pclose() {
    if (state != DEV_OPEN) {
        cout << RED << "close " << dev_name <<" fail, is not open" << NONE << endl;
        return false;
    }
    close(fd);
    state = DEV_CLOSE;
    cout << BLUE << "close " << dev_name <<" suecess" << NONE << endl;
    return true;

}

bool DEV::set_channel(unsigned char ch) {
    unsigned char buf_tmp[] = {0xAA, 0x55, 0x00, 0x02, 0x75, 0x75, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned short crc16_tmp;
    buf_tmp[3] = ch;
    crc16_tmp = parse_crc16(buf_tmp, sizeof(buf_tmp));
    buf_tmp[6] = crc16_tmp&0xff;
	buf_tmp[7] = (crc16_tmp>>8)&0xff;

    printf(GREEN"[set channel: 0x%x]", ch);
    for (int i = 0; i < sizeof(buf_tmp); i++) {
        // cout << " " << hex << write_buf[i];

        printf("0x%x ", buf_tmp[i]);
    }
    // cout << NONE << endl;
    printf(NONE"\n");

    if ((write(fd, buf_tmp, sizeof(buf_tmp))) < 0) {
        perror("uart set channel");
        return false;
    }

    return true;
}

bool DEV::start_send_test(int count) {
    unsigned char write_buf[] = {0xaa, 0x55, 0x0f, 0x06, 0xda, 0xda, 0xa3, 0x71, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03
                                , 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
   
    while (count-- > 0) {
        // cout  << GREEN << "[send:]";
        printf(GREEN"[send:]");
        for (int i = 0; i < sizeof(write_buf); i++) {
            // cout << " " << hex << write_buf[i];

            printf("0x%x ", write_buf[i]);
        }
        // cout << NONE << endl;
        printf(NONE"\n");

        if ((write(fd, write_buf, sizeof(write_buf))) < 0) {
            perror("uart send");
            return false;
        }
        sleep(1);
    }
    
    return true;
}

void DEV::recv_thread() {
    int size;
    unsigned char buf[1024] = {0}; 
    
    while (state != DEV_OPEN) sleep(1);
   
    while (1) {
        // ??????
        size = read(fd, buf, 1024);
		if (size <= 0) {
			perror("read");
		} else {
            for (int i = 0; i < size; i++) {
                recv_buff.push_back(buf[i]);
            }
        }

        // ??????
        while (recv_buff.size() >= 12) {
            if (recv_buff[0] != 0xAA || recv_buff[1] != 0x55) {
                recv_buff.pop_front();
                continue;
            }

            // ????????????
            unsigned char data_len = recv_buff[2];
            if (recv_buff.size() < ( data_len + 12) ) {
                DBG(YELLOW"Size short, need %d but now %d \n"NONE, ( data_len + 5 + 2), recv_buff.size());
                break;
            }

            // ??????
            unsigned char buf_tmp[512] = {0};
            for (int i = 0; i < data_len + 12; i++) {
                buf_tmp[i] = recv_buff[i];
            }
            buf_tmp[6] = 0x00;
            buf_tmp[7] = 0x00;
            unsigned short crc16_tmp = parse_crc16(buf_tmp, data_len + 12);
            
            if ((crc16_tmp&0xff) != recv_buff[6] || ((crc16_tmp>>8)&0xff) != recv_buff[7]) {
                DBG(YELLOW"CRC Error, CRC Need: 0x%x 0x%x, But now: 0x%x 0x%x\n"NONE, 
                        (crc16_tmp&0xff), ((crc16_tmp>>8)&0xff), recv_buff[6], recv_buff[7]);
                recv_buff.pop_front();
                continue;
            }

            /* ???????????? */
            vector<unsigned char> tmp;
            for (int i = 0; i < data_len + 5 + 2; i++) {
                tmp.push_back(recv_buff[0]);
                recv_buff.pop_front();
            }
            _mtx.lock();
            frame.push_back(tmp);
            _mtx.unlock();
        }

		std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 100ms
    }
    
}

bool DEV::start_recv_test(int time) {
    if (_thread != nullptr) {
        _thread = new thread(&DEV::recv_thread, this);
    }
    
    time *= 10;
    while (time) {

        // ????????????
        _mtx.lock();
        if (!frame.empty()) {
            vector<unsigned char> tmp = frame.front();
            printf(GREEN"[recv: %d]", tmp.size());
            for (int i = 0; i < tmp.size(); i ++) {
                printf(" 0x%x", tmp[i]);
            }
            printf(NONE"\n");
            frame.erase(frame.begin());
        }
        _mtx.unlock();

        time--;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 100ms
    }

    return true;
}

int DEV::get_version(void) {

    // ??????????????????
    if (_thread != nullptr) {
        _thread = new thread(&DEV::recv_thread, this);
    }

    // ????????????
    unsigned char send[] = {0x57, 0x59, 0x02, 0x00, 0x0C, 0xAA, 0x55, 0x00, 0x02 
                            , 0x75, 0x75, 0x4F, 0x06, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xAA};
    int size;
    unsigned char buf[1024] = {0};

    if ((write(fd, send, sizeof(send))) < 0) {
        perror("get version send");
        return 0;
    }

    sleep(1);	

    _mtx.lock();
    if (!frame.empty()) {
        cout << RED << "Read version err, not ACK" << NONE << endl;
        _mtx.unlock();
        return 1;
    } else {
        while (!frame.empty()) {
            vector<unsigned char> tmp = frame.front();
            if (tmp.size() > 6 && tmp[4] == 0x75 && tmp[5] == 0x75) {
                printf(GREEN"Recv verison frame:");
                for (int i = 0; i < tmp.size(); i++) {
                    printf(" 0x%x", tmp[i]);
                }
                printf(NONE"\n");
                _mtx.unlock();
                return 0;
            }
        }
    }

    cout << RED << "Can Not Recv Version Frame" << NONE << endl;

    return 1;
}