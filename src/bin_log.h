#ifndef BIN_LOG_H
#define BIN_LOG_H

#include <fstream>
#include <array>
#include <string>
#include <chrono>

namespace arwain {

enum filetypes {
    accelwrite,
    accelread,
    gyrowrite,
    gyroread,
    magwrite,
    magread
};

enum filemodes {
    read,
    write
};

class BinLog
{
    private:
        std::string location;
        std::ofstream handle;
        int filemode = arwain::read;
        long long seek = 0;
    
    public:
        // Constructors
        BinLog(std::string filename, int filetype);
        
        // Operators
        arwain::BinLog & operator<<(std::array<double, 3> vals);
        arwain::BinLog & operator<<(unsigned long long val);
        arwain::BinLog & operator<<(int val);
        arwain::BinLog & operator<<(std::chrono::_V2::system_clock::time_point time);
        
        // File IO/control functions.
        int close();
        long long getFileSize();
};

} // End of namespace arwain.

#endif
