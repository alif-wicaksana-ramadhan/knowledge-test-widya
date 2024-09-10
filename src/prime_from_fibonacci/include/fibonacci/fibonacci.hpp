#ifndef __FIBONACCI__
#define __FIBONACCI__

#include <vector>

using namespace std;

class Fibonacci
{
private:
    int series_length;

public:
    Fibonacci(int n);
    ~Fibonacci();
    vector<int> retrieveFibonacci();
};

#endif