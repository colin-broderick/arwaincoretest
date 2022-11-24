#ifndef _ARWAIN_T265_HPP
#define _ARWAIN_T265_HPP

#include <array>
#include <librealsense2/rs.hpp>

class T265
{
    public: // methods
        T265();
        ~T265();
        std::array<double, 3> get_position() const;
    
    public: // variables
        rs2::pipeline* pipe;

    private: // methods

    private: // variables
};

#endif
