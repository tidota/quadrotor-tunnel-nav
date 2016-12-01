#include <iostream>
#include <iomanip>

#include "sub.hpp"

using namespace std;

void subfunc(int index)
{
	cout << "This is subfunc. Index " << setw(5) << index << endl;
}

void printInTerm(const char *str)
{
    cout << str << endl;
}

