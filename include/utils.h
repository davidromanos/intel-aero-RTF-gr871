#include <inttypes.h>
#include <stdio.h>

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdarg.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>
using namespace std;

void logToFile(const char *file, const char *format, ...);
vector<vector<double> > load_csv (const string &path);
