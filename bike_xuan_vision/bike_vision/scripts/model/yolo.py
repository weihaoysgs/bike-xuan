from model.CSPdarknet53_tiny import DarkNet53Tiny
from model.CSPdarknet53_tiny import BasicConv
import torch


class Upsample(torch.nn.Module):
    def __init__(self, in_channels, out_channels):
        super(Upsample, self).__init__()

        self.upsample = torch.nn.Sequential(
            BasicConv(in_channels, out_channels, 1),
            torch.nn.Upsample(scale_factor=2, mode='nearest')
        )

    def forward(self, x, ):
        x = self.upsample(x)
        return x



# YoloV4 Output
def YoloHead(filters_list, in_filters):
    m = torch.nn.Sequential(
        BasicConv(in_channels=in_filters, out_channels=filters_list[0], kernel_size=3),
        torch.nn.Conv2d(filters_list[0], filters_list[1], 1),
    )
    return m


class YoloBody(torch.nn.Module):
    def __init__(self, anchors_mask, num_classes, phi=0, pretrained=False):
        super(YoloBody, self).__init__()
        self.phi = phi
        self.backbone = DarkNet53Tiny(pretrained=pretrained)
        self.conv_for_P5 = BasicConv(512, 256, 1)
        self.yolo_headP5 = YoloHead([512, len(anchors_mask[0]) * (5 + num_classes)], 256)
        self.upsample = Upsample(256, 128)
        self.yolo_headP4 = YoloHead([256, len(anchors_mask[1]) * (5 + num_classes)], 384)

    def forward(self, x):
        feat1, feat2 = self.backbone(x)
        # 13,13,512 -> 13,13,256
        P5 = self.conv_for_P5(feat2)
        # 13,13,256 -> 13,13,512 -> 13,13,255
        out0 = self.yolo_headP5(P5)
        # 13,13,256 -> 13,13,128 -> 26,26,128
        P5_Upsample = self.upsample(P5)
        # 26,26,256 + 26,26,128 -> 26,26,384
        P4 = torch.cat([P5_Upsample, feat1], axis=1)
        # 26,26,384 -> 26,26,256 -> 26,26,255
        out1 = self.yolo_headP4(P4)
        return out0, out1
