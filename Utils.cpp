// Copyright 2025 Mimir Reynisson
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the “Software”),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "pd/Utils.h"
#ifndef ARDUINO
#include <cmath>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <sys/select.h>
#include <string>
#endif

namespace pd {

int readKeyIfAvailable() {
#ifdef ARDUINO
	if (Serial.available()) {
		return Serial.read();
	}
#else
    fd_set readfds;
    struct timeval timeout;
    int retval;
    char ch;

    // Clear the set ahead of time
    FD_ZERO(&readfds);

    // Add our descriptor to the set
    FD_SET(STDIN_FILENO, &readfds);

    // Set timeout to 0, which makes select non-blocking
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    // See if there is any data available to read from stdin
    retval = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
    return (retval > 0 && read(STDIN_FILENO, &ch, 1) == 1) ? ch : -1;
#endif
    return 0;
}

void setNonCanonicalMode(bool enable) {
#ifndef ARDUINO
    static bool initialized;
    static struct termios oldt, newt;

    if (enable) {
        // Get the terminal settings
        tcgetattr(STDIN_FILENO, &oldt);
        // Copy settings to newt
        newt = oldt;
        // Disable canonical mode and local echo
        newt.c_lflag &= ~(ICANON | ECHO);
        // Set the new settings immediately
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        initialized = true;
    } else if (initialized) {
        // Restore the old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
#endif
}

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

int normalize(int degrees)
{
    degrees = fmod(degrees, 360);
    if (degrees < 0)
        degrees += 360;
    return degrees;
}

int shortestDistance(int origin, int target)
{
    int result = 0.0;
    int diff = fmod(fmod(abs(origin - target), 360), 360);

    if (diff > 180)
    {
        //There is a shorter path in opposite direction
        result = (360 - diff);
        if (target > origin)
            result *= -1;
    }
    else
    {
        result = diff;
        if (origin > target)
            result *= -1;
    }
    return result;
}

}
