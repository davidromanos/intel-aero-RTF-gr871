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
#include <sys/stat.h>
using namespace std;

void logToFile(const char *file, const char *format, ...);
vector<vector<double> > load_csv (const string &path);
void getLogFilePath(char * pathBuffer, const char * suffix);
void prepareLogFile(ofstream * fileObject, const char * filePrefix);
void logAppendTimestamp(ofstream &fileObject, ros::Duration time);
void logAppendTimestampNow(ofstream &fileObject);
