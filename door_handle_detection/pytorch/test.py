import torch
from torch import nn
from torch.utils.data import DataLoader
from tqdm import tqdm
import numpy as np

from heatmap_model import HeatmapModel
from dataset import loadAllData, DoorDataGenerator, showImageLabels

batch_size = 30

model = HeatmapModel().cuda()
checkpoint = torch.load('models/my.pt')
model.load_state_dict(checkpoint['model'])
model.eval()

# Loss
criterion = nn.MSELoss().cuda()

all_images, all_labels = loadAllData()
test_dataset = DoorDataGenerator(all_images, all_labels, train=False, shuffle=False)
test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

for i, (images, labels) in enumerate(tqdm(test_loader)):
    images = images.cuda()
    labels = labels.cuda()
    outputs = model(images)

    loss = criterion(outputs, labels)

    for i in range(len(images)):
        showImageLabels(images[i], outputs[i])

    break
