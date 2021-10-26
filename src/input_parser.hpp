#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

#include <vector>
#include <string>

namespace arwain
{
    class InputParser
    {
        private:
            std::vector<std::string> tokens;

        public:
            InputParser(int &argc, char **argv);
            const std::string& getCmdOption(const std::string &option) const;
            bool contains(const std::string &option) const;
    };
}

#endif
