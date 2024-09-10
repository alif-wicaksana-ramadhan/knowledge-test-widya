#ifndef __PRIME__
#define __PRIME__

#include <vector>

using namespace std;

class Prime
{
public:
    Prime();
    ~Prime();
    int calculateThePrime(vector<int> series);

private:
    bool isPrime(int number);
};

#endif