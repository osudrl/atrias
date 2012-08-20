/*
 * timertest.cpp
 *
 *  Created on: Jul 17, 2012
 *      Author: drl
 */

#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>

struct timeval tv;
struct tm *tm;

int64_t getMicroSecs() {
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    return ((int64_t)tm->tm_sec)*1000000 + ((int64_t)tv.tv_usec);
}

int main (int argc, char **argv) {
    int64_t timeVar;
    int diff;

    while (true) {
    timeVar = getMicroSecs();
        for(int i = 0; i < 99999; i++)
            getMicroSecs();
        diff = (int)(getMicroSecs() - timeVar);
        printf("100,000 calls take %i uS\n", diff);
        printf("1 call takes %i us\n", diff/100000);
        printf("---------------------------\n");
    }
}
