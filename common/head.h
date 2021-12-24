#ifndef _HEAD_H
#define _HEAD_H

#include<iostream>
#include<getopt.h>
#include "color.h"
#include "dev.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <assert.h>
#include<pthread.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#ifdef _D
#define DBG(fmt, args...) printf(fmt, ##args)
#else
#define DBG(fmt, args...)
#endif

#endif