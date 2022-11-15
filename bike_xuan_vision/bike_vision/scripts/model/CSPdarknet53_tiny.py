import math

import torch
import torch.nn as nn


# Conv2d + BatchNorm2d + LeakyReLU
class BasicConv(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride=1):
        super(BasicConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size, stride, kernel_size // 2, bias=False)
        self.bn = nn.BatchNorm2d(out_channels)
        self.activation = nn.LeakyReLU(0.1)

    def forward(self, x):
        return self.activation(self.bn(self.conv(x)))


class ResidualBlockBody(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(ResidualBlockBody, self).__init__()
        self.out_channels = out_channels
        self.conv1 = BasicConv(in_channels, out_channels, 3)
        self.conv2 = BasicConv(out_channels // 2, out_channels // 2, 3)
        self.conv3 = BasicConv(out_channels // 2, out_channels // 2, 3)
        self.conv4 = BasicConv(out_channels, out_channels, 1)
        self.maxpool = nn.MaxPool2d([2, 2], [2, 2])

    def forward(self, x):
        x = self.conv1(x)
        route = x
        c = self.out_channels
        x = torch.split(x, c // 2, dim=1)[1]
        x = self.conv2(x)
        route1 = x
        x = self.conv3(x)
        x = torch.cat([x, route1], dim=1)
        x = self.conv4(x)
        feat = x
        x = torch.cat([route, x], dim=1)
        x = self.maxpool(x)
        return x, feat


class CSPDarkNet(nn.Module):
    def __init__(self):
        super(CSPDarkNet, self).__init__()
        # 416,416,3 -> 208,208,32 -> 104,104,64
        self.conv1 = BasicConv(3, 32, kernel_size=3, stride=2)
        self.conv2 = BasicConv(32, 64, kernel_size=3, stride=2)
        # 104,104,64 -> 52,52,128
        self.resblock_body1 = ResidualBlockBody(64, 64)
        # 52,52,128 -> 26,26,26
        self.resblock_body2 = ResidualBlockBody(128, 128)
        # 26,26,256 -> 13,13,512
        self.resblock_body3 = ResidualBlockBody(256, 256)
        # 13,13,512 -> 13,13,512
        self.conv3 = BasicConv(512, 512, kernel_size=3)
        self.num_features = 1
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
                m.weight.data.normal_(0, math.sqrt(2. / n))
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()

    def forward(self, x):
        # 416,416,3 -> 208,208,32 -> 104,104,64
        x = self.conv1(x)
        x = self.conv2(x)
        # 104,104,64 -> 52,52,128
        x, _ = self.resblock_body1(x)
        # 52,52,128 -> 26,26,256
        x, _ = self.resblock_body2(x)
        # 26,26,256 -> x:13,13,512
        #           -> feat1:26,26,256
        x, feat1 = self.resblock_body3(x)
        # 13,13,512 -> 13,13,512
        x = self.conv3(x)
        feat2 = x
        return feat1, feat2


def DarkNet53Tiny(pretrained, **kwargs):
    model = CSPDarkNet()
    if pretrained:
        model.load_state_dict(torch.load(
            "/home/wangzihanggg/code_space/bike_vision_ws/src/bike_vision/scripts/pth"
            "/CSPdarknet53_tiny_backbone_weights.pth"))
    return model
