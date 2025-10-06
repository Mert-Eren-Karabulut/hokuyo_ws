#include "colorhelper.h"

HSV rgbToHsv(RGB in)
{
    HSV         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min < in.b ? min : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max > in.b ? max : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    }
    else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if (in.r >= max)                           // > is bogus, just keeps compilor happy
        out.h = (in.g - in.b) / delta;        // between yellow & magenta
    else
        if (in.g >= max)
            out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if (out.h < 0.0)
        out.h += 360.0;

    return out;
}

float hsvDist(const HSV& p1, const HSV& p2)
{
    /*
    float dh = std::min(fabs(p1.h - p2.h), 360 - fabs(p1.h - p2.h)) / 180.0;
    float ds = fabs(p1.s - p2.s);
    float dv = fabs(p1.v - p2.v) / 255.0;
    return   sqrt(dh * dh + ds * ds + dv * dv);
    */
    float h_diff = std::min(abs(p1.h - p1.h), 1 - abs(p1.h - p1.h));
    return std::min(p1.s, p2.s) * abs(h_diff) + abs(p1.s - p2.s) + abs(p1.v - p2.v);
}