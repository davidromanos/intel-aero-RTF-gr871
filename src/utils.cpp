#include "utils.h"

void logToFile(const char *file, const char *format, ...)
{
    va_list args;
    char buffer[2048];
    unsigned int i = 0;
    //Appends to a file at the end of the file. The file is created if it does not exist.
    FILE *fp = fopen (file, "a");



   // start the variadic arguments + lock
    va_start(args, format);



    ros::Time now = ros::Time::now();


    u_int32_t s = now.sec;
    u_int32_t ns = now.nsec;
    fprintf(fp, "%u,%u,", s, ns/1000000);

    // print the data
    vsprintf(buffer, format, args);
    for (i = 0; i < strlen(buffer); ++i)
    {

        fprintf(fp, "%c", buffer[i]);
        if (buffer[i] == '\n')
        {
            fprintf(fp, "\t");
        }
    }
    // unlock + clean
    fprintf(fp, "\n");
    va_end(args);

    fclose(fp);

}

void getLogFilePath(char * pathBuffer, const char * suffix) {
  char logPrefix[50];
  time_t rawtime = time(0);
  tm *now = localtime(&rawtime);

  if(rawtime != -1) {
     strftime(logPrefix, 50, "%y%m%d_%H%M%S",now);
     strcat(pathBuffer, logPrefix);
     strcat(pathBuffer, "_");
     strcat(pathBuffer, suffix);
     strcat(pathBuffer, ".txt");
  } else {
      strcat(pathBuffer, "TimeErr_");
      strcat(pathBuffer, suffix);
      strcat(pathBuffer, ".txt");
  }
}

/*
FILE * prepareLogFile(char * pathBuffer, const char * filePrefix)
{
    char * home_dir = getenv("HOME");
    char * log_dir = strcat(home_dir, "/logs/");
    getLogFilePath(log_dir, pathBuffer, filePrefix);

    FILE * logFile;

    // creates the directory with specified modes
    mkdir(log_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    // used the strcat() to generate the file-path.
    logFile = fopen(pathBuffer, "a+");

    return logFile;
}*/

void prepareLogFile(ofstream * fileObject, const char * filePrefix)
{
    char pathBuffer[200];
    strcpy(pathBuffer, getenv("HOME"));
    strcat(pathBuffer, "/logs/");

    /*creates the directory with specified modes*/
    mkdir(pathBuffer, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    getLogFilePath(pathBuffer, filePrefix);
    //ofstream * logFile = new ofstream(pathBuffer, ios::out | ios::app | ios::binary);
    fileObject->open(pathBuffer, ios::out | ios::ate);
}

void logAppendTimestamp(ofstream &fileObject, ros::Duration time)
{
    fileObject << time.sec << "." << setfill('0') << setw(3) << time.nsec / 1000000 << ", ";
}

void logAppendTimestampNow(ofstream &fileObject)
{
    ros::Time now = ros::Time::now();
    fileObject << now.sec << "." << setfill('0') << setw(3) << now.nsec / 1000000 << ", ";
}

vector<vector<double> > load_csv (const string &path) {
    ifstream indata;
    indata.open(path.c_str());
    string line;
    vector<vector<double> > rowVectors;
    vector<double> values;
    uint rows = 0;
    while (getline(indata, line)) {
        stringstream lineStream(line);
        string cell;
        while (getline(lineStream, cell, ',')) {
            values.insert(values.end(), stod(cell));
        }
        rowVectors.insert(rowVectors.end(), values);
        values.clear();
        ++rows;
    }
    return rowVectors;
}
