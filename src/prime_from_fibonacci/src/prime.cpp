#include "prime/prime.hpp"

#include <vector>
#include <cmath>

using namespace std;

Prime::Prime()
{
}

Prime::~Prime()
{
}

bool Prime::isPrime(int number)
{
    if (number <= 1)
    {
        return false;
    }

    for (int i = 2; i <= sqrt(number); i++)
    {
        if (number % i == 0)
        {
            return false;
        }
    }

    return true;
}

int Prime::calculateThePrime(vector<int> series)
{
    int total_prime = 0;

    for (int i = 0; i < series.size(); i++)
    {
        if (isPrime(series[i]))
        {
            total_prime++;
        }
    }

    return total_prime;
}