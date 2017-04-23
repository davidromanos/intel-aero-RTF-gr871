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
