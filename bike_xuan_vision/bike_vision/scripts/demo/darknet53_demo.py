#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
sys.path.append("..")
from model.CSPdarknet53_tiny import DarkNet53Tiny

if __name__ == '__main__':
    model = DarkNet53Tiny(pretrained=True)
    print(model)
