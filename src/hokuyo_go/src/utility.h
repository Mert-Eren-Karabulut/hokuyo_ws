#ifndef EXPLORATION_UTILITY_H
#define EXPLORATION_UTILITY_H

#include <iostream>
#include <vector>
class Conncomp1d{

public:

    int binaryImage[10000];

    double compwidth[10000];

    double centroid[10000];

    int compNumber;

    int count;

    int i;

    int cx;

    int findcompNumber();

    void findcompwidth();

    void findcentroid();

    int SetValues1(int *p1, int count);

};

class StdDeviation
{

private:
    int count;

    double value[10000];

    double mean;

public:

    double CalculateMean();

    double CalculateVariane();

    double CalculateSampleVariane();

    int SetValues(std::vector<float>p, int count);
     
    double GetStandardDeviation();

    double GetSampleStandardDeviation();

};

#endif // UTILITY_H