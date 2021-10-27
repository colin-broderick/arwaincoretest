#ifndef _GREEVE_INPUT_PARSER_HPP
#define _GREEVE_INPUT_PARSER_HPP

#include <vector>
#include <string>

class InputParser
{
    private:
        std::vector<std::string> tokens;

    public:
        InputParser(int &argc, char **argv);
        const std::string& getCmdOption(const std::string &option) const;
        bool contains(const std::string &option) const;
};

#endif
