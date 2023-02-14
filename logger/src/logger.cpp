#include "logger.hpp"

/** \brief Construct Logger and open the relevant file.
 * \param file_name The full path of the file to write to.
*/
arwain::Logger::Logger(const std::string& file_name)
{
    this->filename = file_name;
    this->output_file.open(this->filename);
}

/** \brief The destructor closes the file handle in case it wasn't already done. */
arwain::Logger::~Logger()
{
    this->close();
}

/** \brief Open the output file in a Logger if it is not currently open.
 * \param file_name The full path of the file to write to.
 * \return False if a file handle is already open, true if the file handle is opened successfully.
 */
bool arwain::Logger::open(const std::string& file_name)
{
    if (output_file.is_open())
    {
        return false;
    }
    this->filename = file_name;
    output_file.open(this->filename);
    return true;
}

/** \brief Close output file to prevent further writes.
 * \return True if the file handle is successfully closed, False if the file handle wasn't open
 * so cannot be closed.
 */
bool arwain::Logger::close()
{
    std::lock_guard<std::mutex> lock{guard};
    if (output_file.is_open())
    {
        output_file.close();
        return true;
    }
    else
    {
        return false;
    }
}

/** \brief Check is the output file is currently open.
 * \return True of the file handle is currently open.
*/
bool arwain::Logger::is_open() const
{
    return output_file.is_open();
}

/** \brief Get the file name that the Logger will write to.
 * \return The file name of the currently open file handle.
*/
std::string arwain::Logger::get_filename() const
{
    return filename;
}
