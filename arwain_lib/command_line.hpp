#ifndef _ARWAIN_COMMAND_LINE_HPP
#define _ARWAIN_COMMAND_LINE_HPP

/** \brief Receives input from stdin or equivalent socket and executes commands, switches modes, etc. */
namespace ArwainCLI
{
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode, std::function<void()> func);
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
    bool shutdown();
    void join();
    bool init();
}

#endif
