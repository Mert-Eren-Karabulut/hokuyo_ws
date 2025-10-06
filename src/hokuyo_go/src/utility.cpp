#include "utility.h"

#include <math.h>


int Conncomp1d::findcompNumber() {
    compNumber=0;
    for (i = 0; i < count; i++){
        if(i==0){
            if (binaryImage[i]==1)
                compNumber = compNumber + 1;
        }
        if(i!=0) {
            if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                compNumber = compNumber + 1;
            }
        }
    }
    return compNumber;
}


void Conncomp1d::findcompwidth() {
    cx = -1;

    for (i = 0; i < count; i++){
        if(i==0){
            if (binaryImage[0]==1) {
                cx = cx + 1;
                compwidth[cx] = 1;
            }
        }

        else {

            if (binaryImage[i]==1 && binaryImage[i-1]==1) {
                compwidth[cx] = compwidth[cx] + 1;
            }

            if (binaryImage[i]==0 && binaryImage[i-1]==1) {
                cx = cx + 1;
                compwidth[cx] = 0;
            }

            if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                if (cx==-1) {
                    cx = 0;
                }

                compwidth[cx] = 1;
            }
        }
    }
}

void Conncomp1d::findcentroid() {
    cx=-1;

    for (i = 0; i < count; i++){
        if(i==0){
            if (binaryImage[0]==1) {
                cx = cx + 1;
                centroid[cx] = 0;
            }
        }

        else {
            if (binaryImage[i]==1 && binaryImage[i-1]==1) {
                centroid[cx] = centroid[cx] + 0.5;
            }

            if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                cx = cx + 1;
                centroid[cx]=i;
            }
        }
    }
}

int Conncomp1d::SetValues1(int *p1, int count)
{
    if(count > 2000) return -1;

    this->count = count;

    for(int i = 0; i < count; i++)
        binaryImage[i] = p1[i];
    return 0;
}

double StdDeviation::CalculateMean()
{
    double sum = 0;
    for(int i = 0; i < count; i++){
        if(value[i] < 6.0){
            sum += value[i];
        }
    }
    return (sum / count);
}

double StdDeviation::CalculateVariane()
{
    mean = CalculateMean();

    double temp = 0;
    for(int i = 0; i < count; i++)
    {
        if(value[i] < 6.0){
            temp += (value[i] - mean) * (value[i] - mean) ;
        }
    }
    return temp / count;
}

double StdDeviation::CalculateSampleVariane()
{
    mean = CalculateMean();

    double temp = 0;
    for(int i = 0; i < count; i++)
    {
        temp += (value[i] - mean) * (value[i] - mean) ;
    }
    return temp / (count - 1);
}

int StdDeviation::SetValues(std::vector<float>p, int count)
{
    if(count > 10000)
        return -1;
    this->count = count;
    for(int i = 0; i < count; i++)
        value[i] = p[i];
    return 0;
}

double StdDeviation::GetStandardDeviation()
{
    return sqrt(CalculateVariane());
}

double StdDeviation::GetSampleStandardDeviation()
{
    return sqrt(CalculateSampleVariane());
}