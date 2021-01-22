#include <algorithm>

#include "input_parser.h"

arwain::InputParser::InputParser(int &argc, char **argv)
{
    // Add all tokens in command line to tokens vector.
    for (int i=1; i < argc; ++i)
    {
        this->tokens.push_back(std::string(argv[i]));
    }
}

const std::string& arwain::InputParser::getCmdOption(const std::string &option) const
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

bool arwain::InputParser::contains(const std::string &option) const
{
    // Check for the presense of a sought option.
    return std::find(this->tokens.begin(), this->tokens.end(), option)
        != this->tokens.end();
}
