#include <algorithm>

#include "input_parser.hpp"

/** \brief Constructor.
 * \param argc Number of command line arguments received.
 * \param argv All command line arguments.
 */
InputParser::InputParser(int &argc, char **argv)
{
    // Add all tokens in command line to tokens vector.
    for (int i=1; i < argc; ++i)
    {
        this->tokens.push_back(std::string(argv[i]));
    }
}

/** \brief Get value of a specified command line option.
 * \param option The name of the sought parameter.
 * \return The value of the sought parameter.
 */
const std::string& InputParser::get_cmd_option(const std::string &option) const
{
    // Identify the position of a requested option and return the next token if found.
    std::vector<std::string>::const_iterator itr;
    itr = std::find(this->tokens.begin(), this->tokens.end(), option);
    if (itr != this->tokens.end() && ++itr != this->tokens.end())
    {
        return *itr;
    }
    static const std::string empty_string("");
    return empty_string;
}

/** \brief Check whether input commands containg specified string.
 * \param option The parameter to check for.
 * \return True if the parameter is found, false otherwise.
 */
bool InputParser::contains(const std::string &option) const
{
    // Check for the presense of a sought option.
    return std::find(this->tokens.begin(), this->tokens.end(), option)
        != this->tokens.end();
}
