#include <vector>
#include <cstring>
#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include "rknn_api.h"
#include <sstream>
#include "rknn.h"

using Tensor = std::array<std::array<std::array<std::array<float, 1>, 200>, 6>, 1>;

static void printRKNNTensor(rknn_tensor_attr *attr)
{
    printf("index=%d name=%s n_dims=%d dims=[%d %d %d %d] n_elems=%d size=%d fmt=%d type=%d qnt_type=%d fl=%d zp=%d scale=%f\n", 
            attr->index, attr->name, attr->n_dims, attr->dims[3], attr->dims[2], attr->dims[1], attr->dims[0], 
            attr->n_elems, attr->size, 0, attr->type, attr->qnt_type, attr->fl, attr->zp, attr->scale);
}

static unsigned char *load_model(const char *filename, int *model_size)
{
    FILE *fp = fopen(filename, "rb");
    if (fp == nullptr)
    {
        printf("fopen %s fail!\n", filename);
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    unsigned int model_len = ftell(fp);
    unsigned char *model = (unsigned char*)malloc(model_len);
    fseek(fp, 0, SEEK_SET);
    if (model_len != fread(model, 1, model_len, fp))
    {
        printf("fread %s fail!\n", filename);
        free(model);
        return NULL;
    }
    *model_size = model_len;
    if (fp)
    {
        fclose(fp);
    }
    return model;
}

/* Unused in this example, ignore. */
void load_data_cpp_vector(const std::string& filename, std::vector<std::vector<float>>& outData)
{
    std::ifstream acce_infile(filename + "acce.txt");
    std::ifstream gyro_infile(filename + "gyro.txt");
    std::string acce_line;
    std::string gyro_line;
    std::getline(acce_infile, acce_line); // skip the first line
    std::getline(gyro_infile, gyro_line); // skip the first line
    unsigned int index = 0;
    unsigned int jndex = 0;
    while (std::getline(acce_infile, acce_line) && std::getline(gyro_infile, gyro_line))
    {
        std::vector<float> thisRow;

        std::stringstream acce_lineStream(acce_line);
        std::stringstream gyro_lineStream(gyro_line);
        std::string acce_cell;
        std::string gyro_cell;

        // Read in a line from the gyro file, ignoring timestamp.
        std::getline(gyro_lineStream, gyro_cell, ' ');
        while (std::getline(gyro_lineStream, gyro_cell, ' '))
        {
            float gyro_fcell;
            std::stringstream{gyro_cell} >> gyro_fcell;
            thisRow.emplace_back(gyro_fcell);
            jndex++;
        }
        // Read in a line from the acce file, ignoring timestamp.
        std::getline(acce_lineStream, acce_cell, ' ');
        while (std::getline(acce_lineStream, acce_cell, ' '))
        {
            float acce_fcell;
            std::stringstream(acce_cell) >> acce_fcell;
            thisRow.emplace_back(acce_fcell);
            jndex++;
        }

        outData.emplace_back(thisRow);
        index++;
        jndex = 0;
    }
    std::cout << "Counted " << index << " lines" << std::endl;
}

/* Unused in this example, ignore. */
/* \brief Produces a flat array ordered by channel, i.e. 200, 200, 200, 200, 200, 200. ..., 6 blocks of 200. */
void get_sample_6_200(std::vector<std::vector<float>> inData, float outData[1200], int index)
{
    // std::cout << "Input vector has size (" << inData.size() << ", " << inData[0].size() << ")" << std::endl;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            outData[i*200 + j] = inData[j + index][i];
        }
    }
}

/* Unused in this example, ignore. */
/* \brief Produces a flat array ordered by height, i.e. 6, 6, 6, 6, 6, ..., 200 blocks of 6. */
void get_sample_200_6(std::vector<std::vector<float>> inData, float outData[1200], int index)
{
    // std::cout << "Input vector has size (" << inData.size() << ", " << inData[0].size() << ")" << std::endl;
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            outData[i*6 + j] = inData[i + index][j];
        }
    }
}

void print_data(const Tensor& data)
{
    for (int h = 0; h < 6; h++)
    {
        std::cout << "  ";
        for (int i = 0; i < 8; i++)
        {
            std::cout << data[0][h][i][0] << " ";
        }
        std::cout << "..." << std::endl;
    }
    std::cout << std::endl;
}

int rknn(const std::string& model_filename)
{
    // This block organizes the gyroscope and accelerometer data into a (1, 6, 200, 1) array, to reflect the format of the data
    // when passed to rknn.inference in the Python implementation.
    // The data here is hard coded in raw_data.h, but it has been confirmed that it is identical to the data used by the python inference.
    // ALL_DATA_POINTER is then a pointer to the start of the 1-dimensional array.
    Tensor ALL_DATA;
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ALL_DATA[0][j][i][0] = DATA_GYRO[0][i][j][0];
        }
        for (int j = 3; j < 6; j++)
        {
            ALL_DATA[0][j][i][0] = DATA_ACCEL[0][i][j][0];
        }
    }
    float* ALL_DATA_POINTER = ALL_DATA[0][0][0].data(); 
    std::cout << "Data array has shape (" << ALL_DATA.size() << ", " << ALL_DATA[0].size() << ", " << ALL_DATA[0][0].size() << ", " << ALL_DATA[0][0][0].size() << ")" << std::endl;
    std::cout << "Data sample:" << std::endl;
    print_data(ALL_DATA);



    // Dimensions of data sample.
    const int size[4] = {1, 6, 200, 1};
    const int input_len = size[0] * size[1] * size[2] * size[3];

    // Create RKNN object.
    rknn_context ctx;
    
    int ret;
    int model_len = 0;
    unsigned char *model;

    const char *model_path = model_filename.c_str();
    
    // Load RKNN model
    model = load_model(model_path, &model_len);
    ret = rknn_init(&ctx, model, model_len, 0);
    if (ret < 0)
    {
        std::cout << "rknn_init failed with code " << ret << "\n";
        return -1;
    }

    // Get model io info
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        std::cout << "rknn_query failed with code " << ret << "\n";
        return -1;
    }

    // Get io attributes
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (unsigned int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            std::cout << "rknn_query failed with code " << ret << "\n";
            return -1;
        }
        std::cout << "Input tensor:  ";
        printRKNNTensor(&(input_attrs[i]));
    }
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (unsigned int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            std::cout << "rknn_query failed with code " << ret << "\n";
            return -1;
        }
        std::cout << "Output tensor: ";
        printRKNNTensor(&(output_attrs[i]));
    }

    //////////////////////////// LOOP /////////////////////////////////
    for (int i = 0; i < 1; i++)
    {
        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0;
        inputs[0].buf = ALL_DATA_POINTER;
        inputs[0].size = input_len;
        inputs[0].pass_through = false;          // <-- This gives more plausible but still incorrect values when set to true.
        inputs[0].type = RKNN_TENSOR_FLOAT32;
        inputs[0].fmt = RKNN_TENSOR_NCHW;

        ret = rknn_inputs_set(ctx, 1, inputs);
        if (ret < 0)
        {
            std::cout << "rknn_inputs_set failed with code: " << ret << "\n";
            return -1;
        }

        // Run!
        std::cout << "Running inference" << "\n";
        ret = rknn_run(ctx, nullptr);
        if (ret < 0)
        {
            std::cout << "rknn_run failed with code " << ret << "\n";
            return -1;
        }

        // Get output
        rknn_output outputs[1];
        memset(outputs, 0, sizeof(outputs));
        float results[3];
        outputs[0].want_float = true;
        outputs[0].is_prealloc = true;
        outputs[0].index = 0;
        outputs[0].buf = results;
        outputs[0].size = 3 * sizeof(float);
        ret = rknn_outputs_get(ctx, 1, outputs, NULL);
        if (ret < 0)
        {
            std::cout << "rknn_ouputs_get failed with code " << ret << "\n";
            return -1;
        }

        for (unsigned int j = 0; j < output_attrs[0].n_elems; j++)
        {
            float val = results[j];
            std::cout << j << ": " << val << "\n";
        }

        rknn_outputs_release(ctx, 1, outputs);
    }
    /////////////////////////// END LOOP //////////////////////////////////////////

    if (ctx >= 0)
    {
        rknn_destroy(ctx);
    }
    if (model)
    {
        free(model);
    }
    return 0;
}

/** \brief Tests that the API is functional. Doesn't do anything useful. */
int rktest(const std::string& model_filename)
{
    rknn(model_filename);
    return 0;
}
