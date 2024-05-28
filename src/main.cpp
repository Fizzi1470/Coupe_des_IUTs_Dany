// Copyright (c) 2024 Antoine TRAN TAN

#include "mbed.h"

DigitalOut led1(LED1);


int main()
{
    while (true)
    { // this is the third thread
        led1 = !led1;
        wait_ms(500);
    }
}
