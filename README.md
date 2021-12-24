# rk-uart-test
test for rk uart device





## build

```shell
arm-linux-gnueabihf-g++  espTest.cpp -Icommon/ ./common/dev.cpp -o espTest -static   
```



## run

```shell
./espTest -m1 -t100 -p /dev/ttyUSB5 -b 115200 -c 6
```



## 参数描述

```txt
m 是模式 0 接收 1发送
t 次数/时间
p 设备名
b 波特率
c channel 通道
```

