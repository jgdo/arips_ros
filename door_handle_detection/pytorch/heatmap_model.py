import torch
from torch import nn
import torch.nn.functional as F

class HeatmapModel(nn.Module):
    def __init__(self, input_channels = 3, output_channels = 2, intermediate_channels=[16, 32, 64, 128]):
        super(HeatmapModel, self).__init__()

        self.use_upsample = False

        self.num = len(intermediate_channels)

        conv_chan = [input_channels] + intermediate_channels
        deconv_chan = list(reversed(intermediate_channels)) + [output_channels*2]

        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)

        self.c = nn.ModuleList()
        for lay in range(self.num):
            chan_in = conv_chan[lay]
            chan_out = conv_chan[lay + 1]
            seq = nn.Sequential(nn.Conv2d(chan_in, chan_out, kernel_size=(5, 5), padding=(2, 2), padding_mode='zeros'),
                                nn.BatchNorm2d(chan_out))
            self.c.append(seq)

        if not self.use_upsample:
            self.d = nn.ModuleList()
            for lay in range(self.num):
                chan_in = deconv_chan[lay]
                chan_out = deconv_chan[lay + 1]
                self.d.append(nn.ConvTranspose2d(chan_in, chan_out, kernel_size=(2,2), stride=(2,2)))

        self.comb = nn.ModuleList()
        for lay in range(self.num-1):
            chan = deconv_chan[lay+1]
            in_chan = chan * 3 if self.use_upsample else chan * 2
            seq = nn.Sequential(nn.Conv2d(in_chan, chan, kernel_size=(5, 5), padding=(2, 2), padding_mode='zeros'),
                                nn.LeakyReLU(),
                                nn.Conv2d(chan, chan, kernel_size=(3, 3), padding=(1, 1), padding_mode='zeros'),
                                nn.LeakyReLU())
            self.comb.append(seq)

        final_input_chan =  intermediate_channels[0] if self.use_upsample else output_channels*2
        self.final_conv = nn.Conv2d(final_input_chan, output_channels, kernel_size=(3, 3), padding=(1, 1), padding_mode='zeros')


    def forward(self, x):
        xn = []

        for i in range(self.num):
            x = self.c[i](x)
            x = F.leaky_relu(x)
            x = self.pool(x)

            if i != self.num-1:
                xn.append(x)

        for i in range(0, self.num-1):
            if self.use_upsample:
                x = F.upsample_nearest(x, scale_factor=2)
            else:
                x = self.d[i](x)
            # x += xn[i]
            x = torch.cat([x, xn[self.num-2-i]], dim=1)
            x = self.comb[i](x)

        if self.use_upsample:
            x = F.upsample_nearest(x, scale_factor=2)
        else:
            x = self.d[-1](x)
        x = F.leaky_relu(x)
        x = self.final_conv(x)

        if self.training:
            x = F.leaky_relu(x)
        else:
            x = F.relu(x)

        return x
