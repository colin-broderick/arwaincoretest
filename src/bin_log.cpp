/*
WIP

Binary data logger for the ARWAIN project.

The logger is designed to
* Produce smaller log files by writing binary data rather than strings
* Be more performant at write time by bypassing the need for things like
  string conversions as much as possible.

A side effect is that the data can not be read without the associated reader
and/or knowledge of the file schemas.

No attention has been paid to read time as it is not considered performance-
sensitive. I would expect it to be faster since there will still be no string
work, but it might be slower, and I don't intend to check.

Example:
  An acceleration file is formatted as such:
    The first 20 bytes are a standard set of bytes identifying this file as
    an accelerometer log file, implying the following schema:
    Each subsequent block of 20 bytes is a unique record;

      bytes   | data
    ----------|---------------------------
      0-7     | timestamp as a long long
      8-11    | acceleration x as a float
      12-15   | acceleration y as a float
      16-19   | acceleration z as a float

Files that are flushed and closed correctly will end with an identifying sequence.
The reader should be able to handle a file without the ending sequence, but will
not be expected to handle a file without the starting sequence.
*/

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include "bin_log.h"

/** \brief An example BinLog file is created and written, for testing purposes.
 * \return 1 for success.
 */
int example()
{
    arwain::BinLog log1("data.bin", arwain::accelwrite);
    std::array<float, 3> data1{4.1, 5, 6};
    unsigned long long data2 = 5;
    auto time = std::chrono::system_clock::now();
    // write float array
    log1 << data1;
    //write long long
    log1 << data2;
    //write time
    log1 << time;
    //write all
    log1 << data1 << data2 << time;
    log1.close();
    std::cout << log1.getFileSize() << std::endl;
    return 1;
}

/** \brief Constructor creates/opens the file ready for writing.
 * \param filename Location of file to record into. Will be appended without checks if it already exists.
 * \param filetype The revelant filetype from the arwain::filetypes enum.
 */
arwain::BinLog::BinLog(std::string filename, int filetype)
{
    if (filetype == arwain::accelwrite || filetype == arwain::gyrowrite || filetype == arwain::magwrite)
    {
        filemode = arwain::write;
        location = filename;
        handle.open(location, std::ofstream::out|std::ofstream::app|std::ofstream::binary);
    }
    else
    {
        filemode = arwain::read;
        seek = 0;
    }
}

// Send an array of three floats, e.g. accel data, to the log file.
arwain::BinLog & arwain::BinLog::operator<<(std::array<float, 3> vals)
{
    float buf[3];
    for (unsigned int i = 0; i < vals.size(); i++)
    {
        buf[i] = vals[i];
    }
    handle.write((char*)buf, vals.size()*sizeof(float));
    return *this;
}

// Send a long long (i.e. timestamp) to the file.
arwain::BinLog & arwain::BinLog::operator<<(unsigned long long val)
{
    // TODO This feels hacky.
    unsigned long long buf[1] = {val};
    handle.write((char*)buf, sizeof(unsigned long long));
    return *this;
}

// Send an int to the file.
arwain::BinLog & arwain::BinLog::operator<<(int val)
{
    // TODO This feels hacky.
    int buf[1] = {val};
    handle.write((char*)buf, sizeof(int));
    return *this;
}

arwain::BinLog & arwain::BinLog::operator<<(std::chrono::_V2::system_clock::time_point time)
{
    std::chrono::milliseconds::rep buf[1] = {time.time_since_epoch().count()};
    handle.write((char*)buf, sizeof(std::chrono::milliseconds::rep));
    return *this;
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
