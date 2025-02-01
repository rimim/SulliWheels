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

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdio.h>
#include "pd/Log.h"

using namespace pd;

#ifdef ARDUINO
static void SerialPrint(const char* buf) {
	char ch;
	while ((ch = *buf++) != '\0') {
		if (ch == '\n') {
			Serial.println();
		} else {
			Serial.print(ch);
		}
	}
}
#endif

void Log::warning(const char* str, va_list args) {
#ifdef ARDUINO
	char buf[1024];
	vsnprintf(buf, sizeof(buf), str, args);
	SerialPrint(buf);
#else
	vprintf(str, args);
#endif
}

void Log::error(const char* str, va_list args) {
#ifdef ARDUINO
	char buf[1024];
	vsnprintf(buf, sizeof(buf), str, args);
	SerialPrint(buf);
#else
	vprintf(str, args);
#endif
}

void Log::info(const char* str, va_list args) {
#ifdef ARDUINO
	char buf[1024];
	vsnprintf(buf, sizeof(buf), str, args);
	SerialPrint(buf);
#else
	vprintf(str, args);
#endif
}
