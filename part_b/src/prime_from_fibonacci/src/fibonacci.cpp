#include "fibonacci/fibonacci.hpp"
#include <iostream>

using namespace std;

Fibonacci::Fibonacci(int n)
{
    series_length = n;
}

Fibonacci::~Fibonacci()
{
}

vector<int> Fibonacci::retrieveFibonacci()
{
    vector<int> series(series_length);

    series[0] = 0;
    if (series_length > 1)
    {
        series[1] = 1;
    }

    for (int i = 2; i < series_length; i++)
    {
        series[i] = series[i - 1] + series[i - 2];
    }

    return series;
}