#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
sys.path.append("..")
from model.CSPdarknet53_tiny import ResidualBlockBody
import torch

if __name__ == '__main__':
    input_tensor = torch.randn(1, 32, 640, 480)
    residual_block_body = ResidualBlockBody(32, 32)
    output_tensor = residual_block_body(input_tensor)
    print(output_tensor)
