import torch
from torch import nn
from torch.utils.data import DataLoader
import torch.nn.functional as F
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt

from heatmap_model import HeatmapModel
from dataset import loadAllData, DoorDataGenerator
from export_model import get_pytorch_onnx_model

num_epochs = 150
batch_size = 32
learning_rate = 0.0005

model = HeatmapModel().cuda()

# Loss and optimizer
criterion = nn.MSELoss().cuda()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
# optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate)


all_images, all_labels = loadAllData()
train_dataset = DoorDataGenerator(all_images, all_labels, train=True, shuffle=False)
train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)

#test_dataset = DoorDataGenerator(all_images, all_labels, train=False)
#test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

loss_history = []
try:
    for epoch in range(num_epochs):
        loss_list = []

        t = tqdm(train_loader)
        for i, (images, labels) in enumerate(t):
            images = images.cuda()
            labels = labels.cuda()
            # Run the forward pass
            outputs = model(images)

            # loss = criterion(outputs, labels)

            error = (labels - outputs)**2 # * (labels*0.8 + 0.2)
            loss = torch.mean(error)

            #error_factor = 3

            #outputs = outputs + outputs*labels*(error_factor-1)

            #loss = criterion(outputs, labels*error_factor)

            loss_list.append(loss.item())

            t.set_description("Loss: {:.2e}".format(loss.item()))

            # Backprop and perform Adam optimisation
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        epoch_loss = np.average(loss_list)
        print('Epoch [{}/{}], Loss: {:.2e}'
              .format(epoch + 1, num_epochs, epoch_loss))

        loss_history.append(epoch_loss)
        train_dataset.on_epoch_end()

except KeyboardInterrupt:
    pass

# full_model_path = get_pytorch_onnx_model(model, "heatmap.onnx", (1, 3, 240, 320))

plt.plot(loss_history, label='loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend(loc='upper right')
plt.show()

torch.save({"model": model.state_dict(), "loss_history": loss_history}, 'models/my.pt')
