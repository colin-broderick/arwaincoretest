#ifndef _ARWAIN_EXCEPTIONS_HPP
#define _ARWAIN_EXCEPTIONS_HPP

class NotImplemented : public std::logic_error
{
    public:
        NotImplemented(const std::string& fn_name)
        : std::logic_error("Function not yet implemented: " + fn_name)
        {

        }
};

#endif