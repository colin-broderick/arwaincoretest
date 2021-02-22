#include <cstring>
#include <iostream>
#include "rknn_api.h"

static void printRKNNTensor(rknn_tensor_attr *attr) {
    printf("index=%d name=%s n_dims=%d dims=[%d %d %d %d] n_elems=%d size=%d fmt=%d type=%d qnt_type=%d fl=%d zp=%d scale=%f\n", 
            attr->index, attr->name, attr->n_dims, attr->dims[3], attr->dims[2], attr->dims[1], attr->dims[0], 
            attr->n_elems, attr->size, 0, attr->type, attr->qnt_type, attr->fl, attr->zp, attr->scale);
}

static unsigned char *load_model(const char *filename, int *model_size)
{
    FILE *fp = fopen(filename, "rb");
    if(fp == nullptr) {
        printf("fopen %s fail!\n", filename);
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    unsigned int model_len = ftell(fp);
    unsigned char *model = (unsigned char*)malloc(model_len);
    fseek(fp, 0, SEEK_SET);
    if(model_len != fread(model, 1, model_len, fp)) {
        printf("fread %s fail!\n", filename);
        free(model);
        return NULL;
    }
    *model_size = model_len;
    if(fp) {
        fclose(fp);
    }
    return model;
}

int rknn(char *argv[])
{
    // Dimensions of data sample.
    const int size[4] = {1, 6, 200, 1};
    const int input_len = size[0] * size[1] * size[2] * size[3];

    // Create RKNN object.
    rknn_context ctx;
    
    int ret;
    int model_len = 0;
    unsigned char *model;

    
    const char *model_path = argv[1];
    
    // TODO: Create data sample
    // This has to be read from the global aligned data buffer and converted
    // to the right format.
    unsigned char *sample[input_len] = {(unsigned char*)'a'};

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
    std::cout << "model input num:  " << io_num.n_input << "\n";
    std::cout << "model output num: " << io_num.n_output << "\n";

    // TODO Explain this block
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

    // TODO Explain this block
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

    //////////////////////////// LOOP? /////////////////////////////////
    {
        // TODO Get next data sample


        // Set input data
        // TODO: Lots to check here
        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0;
        inputs[0].type = RKNN_TENSOR_UINT8;
        inputs[0].size = input_len;
        inputs[0].fmt = RKNN_TENSOR_NHWC;
        inputs[0].buf = sample;

        ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
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
        outputs[0].want_float = 1;
        ret = rknn_outputs_get(ctx, 1, outputs, NULL);
        if (ret < 0)
        {
            std::cout << "rknn_ouputs_get failed with code " << ret << "\n";
            return -1;
        }

        // TODO Post process?
        for (unsigned int i = 0; i < output_attrs[0].n_elems; i++) {
            float val = ((float*)(outputs[0].buf))[i];
            std::cout << i << ": " << val << "\n";
        }

        // TODO Put output into a deque buffer.

        // Release rknn_outputs
        // TODO: Not sure what's happening here
        rknn_outputs_release(ctx, 1, outputs);
    }
    /////////////////////////// END LOOP //////////////////////////////////////////

    // Release only after inference is complete.
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
