#include "logger.hpp"

/** \brief Construct Logger and open the relevant file. */
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

/** \brief Open the output file in a Logger if it is not currently open. */
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

/** \brief Close output file to prevent further writes. */
void arwain::Logger::close()
{
    std::lock_guard<std::mutex> lock{guard};
    output_file.close();
}

/** \brief Check is the output file is currently open. */
bool arwain::Logger::is_open() const
{
    return output_file.is_open();
}

/** \brief Get the file name that the Logger will write to. */
std::string arwain::Logger::get_filename() const
{
    return filename;
}
