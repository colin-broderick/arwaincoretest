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
            print("Inference test passed")
        except Exception as e:
            print("Inference test failed:", e)


def main():
    predictor = Predictor(sys.argv[1]+".xml", sys.argv[1]+".bin")
    context = zmq.Context()
    print("Waiting for data connection")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")
    n = 0
    socket.send("accept".encode("ascii"))
    while True:
        message = socket.recv()
        nums  = [float(i) for i in message.decode("ascii").split(",")[:-1]]
        n += 1
        prediction = predictor.predict(nums)
        response = f"{prediction[0]},{prediction[1]}".encode("ascii")
        socket.send(response)


if __name__ == "__main__":
    main()
