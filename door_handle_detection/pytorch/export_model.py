import os

import torch
from torch.autograd import Variable

from heatmap_model import HeatmapModel


def get_pytorch_onnx_model(model, onnx_model_name, input_shape):
    # define the directory for further converted model save
    onnx_model_path = "models"
    # define the name of further converted model

    # create directory for further converted model
    os.makedirs(onnx_model_path, exist_ok=True)

    # get full path to the converted model
    full_model_path = os.path.join(onnx_model_path, onnx_model_name)

    # generate model input
    generated_input = Variable(
        torch.randn((input_shape))
    )

    # model export into ONNX format
    torch.onnx.export(
        model,
        generated_input,
        full_model_path,
        verbose=False,
        input_names=["input"],
        output_names=["output"],
        opset_version=11
    )

    return full_model_path

if __name__ == "__main__":
    model = HeatmapModel(output_channels=1, intermediate_channels=[12, 24, 32, 40])
    num_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print("Number of trained parameters: {}".format(num_params))

    checkpoint = torch.load('models/floor_door_edge_e150_c[12, 24, 32, 40]_loss3.2971648075049e-05.pt')
    model.load_state_dict(checkpoint['model'])
    model.eval()
    full_model_path = get_pytorch_onnx_model(model, "floor_door_edge.onnx", (1, 3, 288, 384))
