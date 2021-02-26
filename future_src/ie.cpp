#include <vector>
#include <iostream>

#include <inference_engine.hpp>
#include <ngraph.hpp>

using namespace InferenceEngine;


template <typename T>
std::ostream & operator << (std::ostream & stream, const std::vector<T> & v)
{
    stream << "[ ";
    for (auto && value : v)
        stream << value << " ";
    return stream << "]";
}

void printParameterValue(const Parameter & value)
{
    if (value.is<bool>())
    {
        std::cout << std::boolalpha << value.as<bool>() << std::noboolalpha << std::endl;
    } else if (value.is<int>())
    {
        std::cout << value.as<int>() << std::endl;
    } else if (value.is<unsigned int>())
    {
        std::cout << value.as<unsigned int>() << std::endl;
    } else if (value.is<float>())
    {
        std::cout << value.as<float>() << std::endl;
    } else if (value.is<std::string>())
    {
        std::string stringValue = value.as<std::string>();
        std::cout << (stringValue.empty() ? "\"\"" : stringValue) << std::endl;
    } else if (value.is<std::vector<std::string> >())
    {
        std::cout << value.as<std::vector<std::string> >() << std::endl;
    } else if (value.is<std::vector<int> >())
    {
        std::cout << value.as<std::vector<int> >() << std::endl;
    } else if (value.is<std::vector<float> >())
    {
        std::cout << value.as<std::vector<float> >() << std::endl;
    } else if (value.is<std::vector<unsigned int> >())
    {
        std::cout << value.as<std::vector<unsigned int> >() << std::endl;
    } else if (value.is<std::tuple<unsigned int, unsigned int, unsigned int> >())
    {
        auto values = value.as<std::tuple<unsigned int, unsigned int, unsigned int> >();
        std::cout << "{ ";
        std::cout << std::get<0>(values) << ", ";
        std::cout << std::get<1>(values) << ", ";
        std::cout << std::get<2>(values);
        std::cout << " }";
        std::cout << std::endl;
    } else if (value.is<std::tuple<unsigned int, unsigned int> >())
    {
        auto values = value.as<std::tuple<unsigned int, unsigned int> >();
        std::cout << "{ ";
        std::cout << std::get<0>(values) << ", ";
        std::cout << std::get<1>(values);
        std::cout << " }";
        std::cout << std::endl;
    } else {
        std::cout << "UNSUPPORTED TYPE" << std::endl;
    }
}


int main(int argc, char *argv[])
{
    const std::string input_model{argv[1]};
    const std::string input_image_path{argv[2]};
    const std::string device_name{argv[3]};

    Core ie;

    // CNNNetwork network = ie.ReadNetwork(input_model, input_model.substr(0, input_model.size() - 4) + ".bin");

    std::vector<std::string> avail = ie.GetAvailableDevices();

    for (auto device : avail)
    {
        std::cout << "Device:  " << device << "\n";
        std::cout << "Metrics: \n";
        std::vector<std::string> supportedMetrics = ie.GetMetric(device, METRIC_KEY(SUPPORTED_METRICS));
        for (auto metricName : supportedMetrics)
        {
            std::cout << "\t" << metricName << ": ";
            printParameterValue(ie.GetMetric(device, metricName));
        }
        CNNNetwork network = ie.ReadNetwork(input_model, input_model.substr(0, input_model.size()-4) + ".bin");

    }

    std::cout << "Hello" << std::endl;
    return 1;
}
