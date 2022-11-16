/*

Data integrity
===============================================================================

These rules are established to ensure that time-sensitive operations can execute
successfully and on time, and that data is never inadvertently changed. Deviation
from these rules should be accompanied by a comment clearly indiciating why.

- Anything accessed by more than one thread should be protected by a mutex.
- Mutex locks should be established only for the duration of a read or write and
  then released immediately. No processing should be done while data is locked.
- Any non-primitive data types should be passed by const reference unless a copy
  is explicitly required.

*/

#include "arwain.hpp"

/** \brief Program entry point.
 * \param argc Number of arguments.
 * \param argv List of arguments.
 */
int main(int argc, char** argv)
{
    return arwain_main(argc, argv);
}
