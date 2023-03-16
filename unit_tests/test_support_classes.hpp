#include <iostream>

class ExposeCout
{
    private:
        std::streambuf* original_cout_buffer;
    public:
        ExposeCout()
        {
            original_cout_buffer = std::cout.rdbuf(nullptr);
        }
        ~ExposeCout()
        {
            std::cout.rdbuf(original_cout_buffer);
        }

};