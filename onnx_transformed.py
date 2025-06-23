import torch
import torch.nn as nn
from models import __models__
import torch.onnx

def convert_to_torchscript():

    model = __models__['StereoModel'](maxdisp=192)
    ckpt = torch.load('./checkpoint/checkpoint.ckpt', map_location='cpu')
    state_dict = ckpt['model']

    new_state_dict = {}
    for k, v in state_dict.items():
        new_key = k.replace("module.", "")
        new_key = new_key.replace("model.", "")
        new_state_dict[new_key] = v

    model.load_state_dict(new_state_dict, strict=False)
    model.cuda().eval()

    w = 640;
    h = 400;
    m = 32;

    wi = int((w // m ) * m);
    hi = int((h // m ) * m);

    limg = torch.randn(1, 3, hi, wi).cuda()
    rimg = torch.randn(1, 3, hi, wi).cuda()

    onnx_path = "StereoModel.onnx"
    torch.onnx.export(
        model,
        (limg, rimg),  # Tuple of inputs
        onnx_path,
        verbose=True,
        input_names=['input1', 'input2'],
        output_names=['output'],
        dynamic_axes={
            'input1': {0: 'batch_size'},
            'input2': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        },
        do_constant_folding=True
    )

if __name__ == "__main__":
    convert_to_torchscript()
