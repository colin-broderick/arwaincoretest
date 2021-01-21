#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include "bin_log.h"

// Example for dev/test
// int main()
// {
//     read("data.bin");
//     arwain::BinLog log1("data.bin", arwain::accelwrite);
//     std::array<float, 3> data{4.1, 5, 6};
//     read("data.bin");
//     log1 << data;
//     log1.close();
//     std::cout << log1.getFileSize() << std::endl;
// }

// Constructor creates/opens the file ready for writing.
arwain::BinLog::BinLog(std::string filename, int filetype)
{
    if (filetype == arwain::accelwrite || filetype == arwain::gyrowrite || filetype == arwain::magwrite)
    {
        filemode = arwain::write;
        location = filename;
        buf_len = 3 * sizeof(float);
        handle.open(location, std::ofstream::out|std::ofstream::app|std::ofstream::binary);
    }
    else
    {
        filemode = arwain::read;
        seek = 0;
    }
}

// Send an array of three floats, e.g. accel data, to the log file.
void arwain::BinLog::operator<<(std::array<float, 3> vals)
{
    float buf[3];
    for (unsigned int i = 0; i < vals.size(); i++)
    {
        buf[i] = vals[i];
    }
    handle.write((char*)buf, vals.size()*sizeof(float));
}

// Send a long long (i.e. timestamp) to the file.
void arwain::BinLog::operator<<(unsigned long long val)
{
    // TODO This feels hacky.
    unsigned long long buf[1] = {val};
    handle.write((char*)buf, sizeof(unsigned long long));
}

// Flush and close the file handle. No further writing will be possible.
int arwain::BinLog::close()
{
    filemode = arwain::read;
    handle.flush();
    handle.close();
    return 1;
}

// Check the current size of the log file.
long long arwain::BinLog::getFileSize()
{
    long long ret;
    struct stat stat_buf;
    int rc = stat(location.c_str(), &stat_buf);
    if (rc == 0)
    {
        ret = stat_buf.st_size;
    }
    else
    {
        ret = -1;
    }
    return ret;
}

// TODO: Read the entire file into some data structure.
int arwain::BinLog::read()
{
    return 1;
}

// TODO: Read one line of the file and advance seek to the next line.
int arwain::BinLog::readLine()
{
    if (seek < getFileSize())
    {

    }
    return 1;
}
