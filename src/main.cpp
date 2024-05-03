#include "arwain/arwain.hpp"
#include "arwain/arwain_bit.hpp"

/** \brief Program entry point.
 * \param argc Number of arguments.
 * \param argv List of arguments.
 */
int main(int argc, char **argv)
{
    auto bit_result = arwain_bit(argc, argv);
    if (bit_result != ArwainBIT::ALL_OK)
    {
        return static_cast<int>(bit_result);
    }
    return static_cast<int>(arwain_main(argc, argv));
}
