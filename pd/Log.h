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

#pragma once

#include <string.h>
#include <stdarg.h>

namespace pd {

class Log {
public:
	bool parse(const char* arg) {
        if (strcmp(arg, "-v") == 0) {
            fVerbose = true;
        } else if (strcmp(arg, "-v:move") == 0) {
            fVerboseMove = true;
        } else if (strcmp(arg, "-v:motor") == 0) {
            fVerboseMotor = true;
        } else if (strcmp(arg, "-v:servo") == 0) {
            fVerboseServo = true;
        } else if (strcmp(arg, "-v:can") == 0) {
            fVerboseCAN = true;
        } else if (strcmp(arg, "-v:i2c") == 0) {
            fVerboseI2C = true;
        } else if (strcmp(arg, "-v:pos") == 0) {
            fVerbosePosition = true;
        } else {
        	return false;
        }
        return true;
	}

	static inline bool isVerbose() {
		return log().fVerbose;
	}

	static inline bool isVerboseCAN() {
		return log().fVerboseCAN;
	}

	static inline bool isVerboseI2C() {
		return log().fVerboseI2C;
	}

	static inline bool isVerboseServo() {
		return log().fVerboseServo;
	}

	static inline bool isVerboseMotor() {
		return log().fVerboseMotor;
	}

	static inline bool isVerboseMove() {
		return log().fVerboseMove;
	}

	static inline bool isVerbosePosition() {
		return log().fVerbosePosition;
	}

	static Log& log() {
		static Log sLog;
		return sLog;
	}

	void warning(const char* str, va_list args);
	void error(const char* str, va_list args);
	void info(const char* str, va_list args);

	void warning(const char* str, ...) {
		va_list args;
	    va_start(args, str);
		warning(str, args);
	    va_end(args);
	}

	void error(const char* str, ...) {
		va_list args;
    	va_start(args, str);
		error(str, args);
	    va_end(args);
	}

	void info(const char* str, ...) {
		va_list args;
	    va_start(args, str);
		info(str, args);
	    va_end(args);
	}

private:
	Log() {}

	bool fVerbose = false;
	bool fVerboseCAN = false;
	bool fVerboseI2C = false;
	bool fVerboseServo = false;
	bool fVerboseMove = false;
	bool fVerboseMotor = false;
	bool fVerbosePosition = false;
};

}

#define PDLOG_WARNING pd::Log::log().warning
#define PDLOG_ERROR pd::Log::log().error
#define PDLOG_INFO pd::Log::log().info
