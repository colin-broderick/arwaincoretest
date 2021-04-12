#!/usr/bin/python3

from __future__ import print_function
import sys
import zmq
import torch
from openvino.inference_engine import IECore, IENetwork


class Predictor:
    def __init__(self, model_xml, model_bin):
        self.ie = IECore()
        self.config = {"VPU_HW_STAGES_OPTIMIZATION": "YES"}
        self.ie.set_config(self.config, "MYRIAD")
        self.xml = model_xml
        self.bin = model_bin
        self.net = IENetwork(model=self.xml, weights=self.bin)
        self.exec_net = self.ie.load_network(self.net, "MYRIAD")
        self.input_blob = next(iter(self.net.inputs))
        self.input_shape = self.net.inputs[self.input_blob].shape
        self.init_test()
    
    def infer_nsc2(self, data):
        result = self.exec_net.infer({self.input_blob: data.cpu().numpy()})
        pred = result["output"][0]
        return torch.Tensor(pred)

    def predict(self, flatdata):
        data = torch.Tensor(flatdata)
        data = torch.reshape(data, (200, 6)).t()
        data = torch.reshape(data, (1, 6, 200))
        prediction = self.infer_nsc2(data)
        return prediction

    def init_test(self):
        """
        Test that inference can be done on random data.
        """
        try:
            data = torch.rand(1, 6, 200)
            sample = self.predict(data)
            print("Inference test passed", file=sys.stderr)
        except Exception as e:
            print("Inference test failed - is the NSC2 accessible?", file=sys.stderr)
            raise e

    def __del__(self):
        del self.net
        del self.exec_net


def main():
    ## TODO Currently, the main program will hang if this quits early due to a failure to get
    ## a handle on the NCS2. Find some way to prevent that.

    ## Get model file location from command line.
    model_xml = sys.argv[1]
    model_bin  = sys.argv[1][:-3] + "bin"

    ## Create a predictor object; this abstracts all the NCS2 stuff.
    try:
        predictor = Predictor(model_xml, model_bin)
    except Exception as e:
        print("Failed to get handle on NCS2", file=sys.stderr)
        raise e
    
    ## Open a socket and send first message.
    context = zmq.Context()
    print("Waiting for data connection")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    socket.send("accept".encode("ascii"))
    inference_counter = 0

    ## Receive a message, process it, and send a response.
    while True:
        message = socket.recv()
        message = message.decode("ascii")

        ## Quit loop and end program if "stop" received from socket.
        if message == "stop":
            break

        ## Otherwise, assume message has the appropriate form and decode it.
        nums = [float(i) for i in message.split(",")[:-1]]

        ## Use NCS2 for inference.
        prediction = predictor.predict(nums)

        ## Form and send response.
        response = f"{prediction[0]},{prediction[1]},{prediction[2]}".encode("ascii")
        socket.send(response)
        inference_counter += 1


if __name__ == "__main__":
    main()
