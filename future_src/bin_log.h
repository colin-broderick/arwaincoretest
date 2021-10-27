#ifndef BIN_LOG_H
#define BIN_LOG_H

#include <fstream>
#include <array>
#include <string>
#include <chrono>

namespace arwain {

enum class filetypes {
    accelwrite,
    accelread,
    gyrowrite,
    gyroread,
    magwrite,
    magread
};

enum class filemodes {
    read,
    write
};

class BinLog
{
    private:
        std::string m_location;
        std::ofstream m_handle;
        int m_filemode = arwain::read;
        long long m_seek = 0;
    
    public:
        // Constructors
        BinLog(const std::string &filename, int filetype);
        
        // Operators
        arwain::BinLog& operator<<(const std::array<double, 3> &vals);
        arwain::BinLog& operator<<(const unsigned long long &val);
        arwain::BinLog& operator<<(const int &val);
        arwain::BinLog& operator<<(const std::chrono::_V2::system_clock::time_point &time);
        
        // File IO/control functions.
        int close();
        long long getFileSize();
};

} // End of namespace arwain.

#endif
