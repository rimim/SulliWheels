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

#include <cmath>
#include <stdint.h>

namespace pd {

class Easing
{
public:
    typedef double (*Method)(double);

    enum
    {
        kLinearInterpolation = 0,
        kContinuous = 1,
        kQuadraticEaseIn = 2,
        kQuadraticEaseOut = 3,
        kQuadraticEaseInOut = 4,
        kCubicEaseIn = 5,
        kCubicEaseOut = 6,
        kCubicEaseInOut = 7,
        kQuarticEaseIn = 8,
        kQuarticEaseOut = 9,
        kQuarticEaseInOut = 10,
        kQuinticEaseIn = 11,
        kQuinticEaseOut = 12,
        kQuinticEaseInOut = 13,
        kSineEaseIn = 14,
        kSineEaseOut = 15,
        kSineEaseInOut = 16,
        kCircularEaseIn = 17,
        kCircularEaseOut = 18,
        kCircularEaseInOut = 19,
        kExponentialEaseIn = 20,
        kExponentialEaseOut = 21,
        kExponentialEaseInOut = 22,
        kElasticEaseIn = 23,
        kElasticEaseOut = 24,
        kElasticEaseInOut = 25,
        kBackEaseIn = 26,
        kBackEaseOut = 27,
        kBackEaseInOut = 28,
        kBounceEaseIn = 29,
        kBounceEaseOut = 30,
        kBounceEaseInOut = 31
    };

    // Modeled after the line y = x
    static double LinearInterpolation(double p)
    {
        return p;
    }

    static double Continuous(double p)
    {
        return p;
    }

    // Modeled after the parabola y = x^2
    static double QuadraticEaseIn(double p)
    {
        return p * p;
    }

    // Modeled after the parabola y = -x^2 + 2x
    static double QuadraticEaseOut(double p)
    {
        return -(p * (p - 2));
    }

    // Modeled after the piecewise quadratic
    // y = (1/2)((2x)^2)             ; [0, 0.5)
    // y = -(1/2)((2x-1)*(2x-3) - 1) ; [0.5, 1]
    static double QuadraticEaseInOut(double p)
    {
        if (p < 0.5)
        {
            return 2 * p * p;
        }
        else
        {
            return (-2 * p * p) + (4 * p) - 1;
        }
    }

    // Modeled after the cubic y = x^3
    static double CubicEaseIn(double p)
    {
        return p * p * p;
    }

    // Modeled after the cubic y = (x - 1)^3 + 1
    static double CubicEaseOut(double p)
    {
        double f = (p - 1);
        return f * f * f + 1;
    }

    // Modeled after the piecewise cubic
    // y = (1/2)((2x)^3)       ; [0, 0.5)
    // y = (1/2)((2x-2)^3 + 2) ; [0.5, 1]
    static double CubicEaseInOut(double p)
    {
        if (p < 0.5)
        {
            return 4 * p * p * p;
        }
        else
        {
            double f = ((2 * p) - 2);
            return 0.5 * f * f * f + 1;
        }
    }

    // Modeled after the quartic x^4
    static double QuarticEaseIn(double p)
    {
        return p * p * p * p;
    }

    // Modeled after the quartic y = 1 - (x - 1)^4
    static double QuarticEaseOut(double p)
    {
        double f = (p - 1);
        return f * f * f * (1 - p) + 1;
    }

    // Modeled after the piecewise quartic
    // y = (1/2)((2x)^4)        ; [0, 0.5)
    // y = -(1/2)((2x-2)^4 - 2) ; [0.5, 1]
    static double QuarticEaseInOut(double p) 
    {
        if (p < 0.5)
        {
            return 8 * p * p * p * p;
        }
        else
        {
            double f = (p - 1);
            return -8 * f * f * f * f + 1;
        }
    }

    // Modeled after the quintic y = x^5
    static double QuinticEaseIn(double p) 
    {
        return p * p * p * p * p;
    }

    // Modeled after the quintic y = (x - 1)^5 + 1
    static double QuinticEaseOut(double p) 
    {
        double f = (p - 1);
        return f * f * f * f * f + 1;
    }

    // Modeled after the piecewise quintic
    // y = (1/2)((2x)^5)       ; [0, 0.5)
    // y = (1/2)((2x-2)^5 + 2) ; [0.5, 1]
    static double QuinticEaseInOut(double p) 
    {
        if (p < 0.5)
        {
            return 16 * p * p * p * p * p;
        }
        else
        {
            double f = ((2 * p) - 2);
            return  0.5 * f * f * f * f * f + 1;
        }
    }

    // Modeled after quarter-cycle of sine wave
    static double SineEaseIn(double p)
    {
        return sin((p - 1) * M_PI_2) + 1;
    }

    // Modeled after quarter-cycle of sine wave (different phase)
    static double SineEaseOut(double p)
    {
        return sin(p * M_PI_2);
    }

    // Modeled after half sine wave
    static double SineEaseInOut(double p)
    {
        return 0.5 * (1 - cos(p * M_PI));
    }

    // Modeled after shifted quadrant IV of unit circle
    static double CircularEaseIn(double p)
    {
        return 1 - sqrt(1 - (p * p));
    }

    // Modeled after shifted quadrant II of unit circle
    static double CircularEaseOut(double p)
    {
        return sqrt((2 - p) * p);
    }

    // Modeled after the piecewise circular function
    // y = (1/2)(1 - sqrt(1 - 4x^2))           ; [0, 0.5)
    // y = (1/2)(sqrt(-(2x - 3)*(2x - 1)) + 1) ; [0.5, 1]
    static double CircularEaseInOut(double p)
    {
        if (p < 0.5)
        {
            return 0.5 * (1 - sqrt(1 - 4 * (p * p)));
        }
        else
        {
            return 0.5 * (sqrt(-((2 * p) - 3) * ((2 * p) - 1)) + 1);
        }
    }

    // Modeled after the exponential function y = 2^(10(x - 1))
    static double ExponentialEaseIn(double p)
    {
        return (p == 0.0) ? p : pow(2, 10 * (p - 1));
    }

    // Modeled after the exponential function y = -2^(-10x) + 1
    static double ExponentialEaseOut(double p)
    {
        return (p == 1.0) ? p : 1 - pow(2, -10 * p);
    }

    // Modeled after the piecewise exponential
    // y = (1/2)2^(10(2x - 1))         ; [0,0.5)
    // y = -(1/2)*2^(-10(2x - 1))) + 1 ; [0.5,1]
    static double ExponentialEaseInOut(double p)
    {
        if (p == 0.0 || p == 1.0)
        {
            return p;
        }
        else if (p < 0.5)
        {
            return 0.5 * pow(2, (20 * p) - 10);
        }
        return -0.5 * pow(2, (-20 * p) + 10) + 1;
    }

    // Modeled after the damped sine wave y = sin(13pi/2*x)*pow(2, 10 * (x - 1))
    static double ElasticEaseIn(double p)
    {
        return sin(13 * M_PI_2 * p) * pow(2, 10 * (p - 1));
    }

    // Modeled after the damped sine wave y = sin(-13pi/2*(x + 1))*pow(2, -10x) + 1
    static double ElasticEaseOut(double p)
    {
        return sin(-13 * M_PI_2 * (p + 1)) * pow(2, -10 * p) + 1;
    }

    // Modeled after the piecewise exponentially-damped sine wave:
    // y = (1/2)*sin(13pi/2*(2*x))*pow(2, 10 * ((2*x) - 1))      ; [0,0.5)
    // y = (1/2)*(sin(-13pi/2*((2x-1)+1))*pow(2,-10(2*x-1)) + 2) ; [0.5, 1]
    static double ElasticEaseInOut(double p)
    {
        if (p < 0.5)
        {
            return 0.5 * sin(13 * M_PI_2 * (2 * p)) * pow(2, 10 * ((2 * p) - 1));
        }
        else
        {
            return 0.5 * (sin(-13 * M_PI_2 * ((2 * p - 1) + 1)) * pow(2, -10 * (2 * p - 1)) + 2);
        }
    }

    // Modeled after the overshooting cubic y = x^3-x*sin(x*pi)
    static double BackEaseIn(double p)
    {
        return p * p * p - p * sin(p * M_PI);
    }

    // Modeled after overshooting cubic y = 1-((1-x)^3-(1-x)*sin((1-x)*pi))
    static double BackEaseOut(double p)
    {
        double f = (1 - p);
        return 1 - (f * f * f - f * sin(f * M_PI));
    }

    // Modeled after the piecewise overshooting cubic function:
    // y = (1/2)*((2x)^3-(2x)*sin(2*x*pi))           ; [0, 0.5)
    // y = (1/2)*(1-((1-x)^3-(1-x)*sin((1-x)*pi))+1) ; [0.5, 1]
    static double BackEaseInOut(double p)
    {
        if (p < 0.5)
        {
            double f = 2 * p;
            return 0.5 * (f * f * f - f * sin(f * M_PI));
        }
        else
        {
            double f = (1 - (2*p - 1));
            return 0.5 * (1 - (f * f * f - f * sin(f * M_PI))) + 0.5;
        }
    }

    static double BounceEaseIn(double p)
    {
        return 1 - BounceEaseOut(1 - p);
    }

    static double BounceEaseOut(double p)
    {
        if (p < 4/11.0)
        {
            return (121 * p * p)/16.0;
        }
        else if (p < 8/11.0)
        {
            return (363/40.0 * p * p) - (99/10.0 * p) + 17/5.0;
        }
        else if (p < 9/10.0)
        {
            return (4356/361.0 * p * p) - (35442/1805.0 * p) + 16061/1805.0;
        }
        return (54/5.0 * p * p) - (513/25.0 * p) + 268/25.0;
    }

    static double BounceEaseInOut(double p)
    {
        if (p < 0.5)
        {
            return 0.5 * BounceEaseIn(p*2);
        }
        else
        {
            return 0.5 * BounceEaseOut(p * 2 - 1) + 0.5;
        }
    }

    static Method get(Method easing) {
        return (easing != nullptr) ? easing : LinearInterpolation;
    }

    static Method get(unsigned i)
    {
        switch (i)
        {
            case kLinearInterpolation:
                return LinearInterpolation;
            case kContinuous:
                return Continuous;
            case kQuadraticEaseIn:
                return QuadraticEaseIn;
            case kQuadraticEaseOut:
                return QuadraticEaseOut;
            case kQuadraticEaseInOut:
                return QuadraticEaseInOut;
            case kCubicEaseIn:
                return CubicEaseIn;
            case kCubicEaseOut:
                return CubicEaseOut;
            case kCubicEaseInOut:
                return CubicEaseInOut;
            case kQuarticEaseIn:
                return QuarticEaseIn;
            case kQuarticEaseOut:
                return QuarticEaseOut;
            case kQuarticEaseInOut:
                return QuarticEaseInOut;
            case kQuinticEaseIn:
                return QuinticEaseIn;
            case kQuinticEaseOut:
                return QuinticEaseOut;
            case kQuinticEaseInOut:
                return QuinticEaseInOut;
            case kSineEaseIn:
                return SineEaseIn;
            case kSineEaseOut:
                return SineEaseOut;
            case kSineEaseInOut:
                return SineEaseInOut;
            case kCircularEaseIn:
                return CircularEaseIn;
            case kCircularEaseOut:
                return CircularEaseOut;
            case kCircularEaseInOut:
                return CircularEaseInOut;
            case kExponentialEaseIn:
                return ExponentialEaseIn;
            case kExponentialEaseOut:
                return ExponentialEaseOut;
            case kExponentialEaseInOut:
                return ExponentialEaseInOut;
            case kElasticEaseIn:
                return ElasticEaseIn;
            case kElasticEaseOut:
                return ElasticEaseOut;
            case kElasticEaseInOut:
                return ElasticEaseInOut;
            case kBackEaseIn:
                return BackEaseIn;
            case kBackEaseOut:
                return BackEaseOut;
            case kBackEaseInOut:
                return BackEaseInOut;
            case kBounceEaseIn:
                return BounceEaseIn;
            case kBounceEaseOut:
                return BounceEaseOut;
            case kBounceEaseInOut:
                return BounceEaseInOut;
        }
        return NULL;
    }
};

}
