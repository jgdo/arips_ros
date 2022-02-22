import torch
from torch import nn
from torch.utils.data import DataLoader
from tqdm import tqdm
import numpy as np

from heatmap_model import HeatmapModel
from dataset import showImageLabels, create_floor_step_loaders
from pytorch.train_floor_step import floor_step_loss_single

batch_size = 30

model = HeatmapModel(output_channels=1, intermediate_channels=[8, 16, 16, 16, 16]).cuda()
checkpoint = torch.load('models/floor_door_edge_e150_c[8, 16, 16, 16, 16]_loss3.983573921934956e-05.pt')
model.load_state_dict(checkpoint['model'])
model.eval()

train_loader, test_loader = create_floor_step_loaders(batch_size=32)


def show_random_test():
    for i, (images, labels, indices) in enumerate(tqdm(test_loader)):
        images = images.cuda()
        outputs = model(images)

        for i in range(len(images)):
            showImageLabels(images[i], outputs[i])

        break


def show_worst_test():
    results = []
    for images, gt_labels in test_loader:
        pred_labels = model(images.cuda()).cpu().detach()
        loss_list = floor_step_loss_single(gt_labels, pred_labels).detach().numpy().tolist()
        pred_list = list(pred_labels.split(1))
        gt_list = list(gt_labels.cpu().split(1))
        img_list = list(images.split(1))
        results += zip(img_list, pred_list, gt_list, loss_list)

    results.sort(key=lambda x: x[3])
    results.reverse()

    for i in range(20):
        showImageLabels(results[i][0], results[i][1])
        showImageLabels(results[i][0], results[i][2])


show_worst_test()
