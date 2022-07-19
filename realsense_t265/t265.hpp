#ifndef _ARWAIN_T265_HPP
#define _ARWAIN_T265_HPP

#include <librealsense2/rs.hpp>

#include "vector3.hpp"

class T265
{
    public: // methods
        T265();
        ~T265();
        Vector3 get_position() const;
    
    public: // variables
        rs2::pipeline* pipe;

    private: // methods

    private: // variables
};

#endif
