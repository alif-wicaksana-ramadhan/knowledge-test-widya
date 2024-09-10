#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "prime/prime.hpp"
#include "fibonacci/fibonacci.hpp"

using namespace std;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR("You must provide an integer argument for robot_speed");
        return 1;
    }

    long int n = atoi(argv[1]);
    Prime prime = Prime();
    Fibonacci fibo = Fibonacci(n);

    vector<int> fib = fibo.retrieveFibonacci();

    for (int i = 0; i < fib.size(); i++)
    {
        cout << fib[i] << " ";
    }
    cout << endl;

    int result = prime.calculateThePrime(fib);

    cout << result << endl;

    return 0;
}