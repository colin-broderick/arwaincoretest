#ifndef BIN_LOG_H
#define BIN_LOG_H

#include <fstream>
#include <array>
#include <string>

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
        unsigned int buf_len;
        // float buf[10];
        std::ofstream handle;
        int filemode = arwain::read;
        long long seek = 0;
    
    public:
        // Constructors
        BinLog(std::string filename, int filetype);
        
        // Operators
        void operator<<(std::array<float, 3> vals);
        void operator<<(unsigned long long val);
        
        // File IO/control functions.
        int close();
        int readLine();
        int read();
        long long getFileSize();
};

} // end of namespace arwain

#endif
