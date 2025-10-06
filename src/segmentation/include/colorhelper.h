#pragma once
#include <math.h> 
#include <algorithm>

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} RGB;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} HSV;

HSV rgbToHsv(RGB in);
float hsvDist(const HSV& p1, const HSV& p2);

