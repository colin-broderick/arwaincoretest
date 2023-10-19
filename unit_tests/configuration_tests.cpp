#include <gtest/gtest.h>

#include <arwain/input_parser.hpp>

#include "arwain/configuration.hpp"

TEST(arwain__Configuration, Configuration_all)
{
    EXPECT_NO_THROW(
        (arwain::Configuration{})
    );

    std::string text0 = "exename";
    std::string text1 = "--lstd";
    std::string text2 = "--noinf";
    std::string text3 = "--nolora";
    std::string text4 = "--nopressure";
    std::string text5 = "--noimu";
    std::string text6 = "--conf";
    std::string text7 = "--conffile";

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

    arwain::InputParser input_parser{count, strings};
    EXPECT_NO_THROW(
        (arwain::Configuration{input_parser})
    );
}

TEST(arwain__Configuration, read_from_file__invalid_file)
{
    std::string text0 = "exename";
    std::string text1 = "--lstd";
    std::string text2 = "--noinf";
    std::string text3 = "--nolora";
    std::string text4 = "--nopressure";
    std::string text5 = "--noimu";
    std::string text6 = "--conf";
    std::string text7 = "--conffile";
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
    arwain::InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    EXPECT_THROW((arwain::ReturnCode::NoConfigurationFile == conf.read_from_file()), std::runtime_error);
}

/** \brief A valid configuration file MUST be provided at the specified location. */
TEST(arwain__Configuration, read_from_file__valid_file)
{
    GTEST_SKIP(); // Figure out a way to provide valid file without assuming file is available. Generate one?

    std::string text0 = "exename";
    std::string text1 = "--lstd";
    std::string text2 = "--noinf";
    std::string text3 = "--nolora";
    std::string text4 = "--nopressure";
    std::string text5 = "--noimu";
    std::string text6 = "--conf";
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
    arwain::InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    EXPECT_TRUE((arwain::ReturnCode::Success == conf.read_from_file()));
}

TEST(arwain__Configuration, replace)
{
    std::string text0 = "exename";
    std::string text1 = "--lstd";
    std::string text2 = "--noinf";
    std::string text3 = "--nolora";
    std::string text4 = "--nopressure";
    std::string text5 = "--noimu";
    std::string text6 = "--conf";
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
    arwain::InputParser input_parser{count, strings};

    arwain::Configuration conf{input_parser};
    EXPECT_NO_THROW(conf.replace("mag_scale_x", 1));
}
