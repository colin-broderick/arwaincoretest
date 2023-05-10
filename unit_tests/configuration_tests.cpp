#include <gtest/gtest.h>

#include "arwain/configuration.hpp"

#include "input_parser.hpp"

TEST(arwain__Configuration, Configuration_all)
{
    EXPECT_NO_THROW(
        (arwain::Configuration{})
    );

    std::string text0 = "exename";
    std::string text1 = "-lstd";
    std::string text2 = "-noinf";
    std::string text3 = "-nolora";
    std::string text4 = "-nopressure";
    std::string text5 = "-noimu";
    std::string text6 = "-conf";
    std::string text7 = "-conffile";

    char* strings[8] = {
        text0.data(),
        text1.data(),
        text2.data(),
        text3.data(),
        text4.data(),
        text5.data(),
        text6.data(),
        text7.data()
    };
    int count = 8;

    InputParser input_parser{count, strings};
    EXPECT_NO_THROW(
        (arwain::Configuration{input_parser})
    );
}

TEST(arwain__Configuration, read_from_file__invalid_file)
{
    std::string text0 = "exename";
    std::string text1 = "-lstd";
    std::string text2 = "-noinf";
    std::string text3 = "-nolora";
    std::string text4 = "-nopressure";
    std::string text5 = "-noimu";
    std::string text6 = "-conf";
    std::string text7 = "-conffile";
    char* strings[8] = {
        text0.data(),
        text1.data(),
        text2.data(),
        text3.data(),
        text4.data(),
        text5.data(),
        text6.data(),
        text7.data()
    };
    int count = 8;
    InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    EXPECT_TRUE((arwain::ReturnCode::NoConfigurationFile == conf.read_from_file()));
}

/** \brief A valid configuration file MUST be provided at the specified location. */
TEST(arwain__Configuration, read_from_file__valid_file)
{
    std::string text0 = "exename";
    std::string text1 = "-lstd";
    std::string text2 = "-noinf";
    std::string text3 = "-nolora";
    std::string text4 = "-nopressure";
    std::string text5 = "-noimu";
    std::string text6 = "-conf";
    std::string text7 = "../arwain.conf";
    char* strings[8] = {
        text0.data(),
        text1.data(),
        text2.data(),
        text3.data(),
        text4.data(),
        text5.data(),
        text6.data(),
        text7.data()
    };
    int count = 8;
    InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    EXPECT_TRUE((arwain::ReturnCode::Success == conf.read_from_file()));
}

TEST(arwain__Configuration, replace)
{
    std::string text0 = "exename";
    std::string text1 = "-lstd";
    std::string text2 = "-noinf";
    std::string text3 = "-nolora";
    std::string text4 = "-nopressure";
    std::string text5 = "-noimu";
    std::string text6 = "-conf";
    std::string text7 = "../arwain.conf";
    char* strings[8] = {
        text0.data(),
        text1.data(),
        text2.data(),
        text3.data(),
        text4.data(),
        text5.data(),
        text6.data(),
        text7.data()
    };
    int count = 8;
    InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    EXPECT_NO_THROW(conf.replace("mag_scale_x", 1));
}

TEST(arwain__Configuration, read_option__invalid_option)
{
    std::string text0 = "exename";
    std::string text1 = "-lstd";
    std::string text2 = "-noinf";
    std::string text3 = "-nolora";
    std::string text4 = "-nopressure";
    std::string text5 = "-noimu";
    std::string text6 = "-conf";
    std::string text7 = "../arwain.conf";
    char* strings[8] = {
        text0.data(),
        text1.data(),
        text2.data(),
        text3.data(),
        text4.data(),
        text5.data(),
        text6.data(),
        text7.data()
    };
    int count = 8;
    InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    double val_storage = 0;
    std::map<std::string, std::string> options;
    EXPECT_THROW(conf.read_option(options, "nonsense", val_storage), std::invalid_argument);
}
