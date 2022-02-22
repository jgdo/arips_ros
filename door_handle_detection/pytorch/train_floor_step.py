import torch
from torch import Tensor
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
from torchsummary import summary

from heatmap_model import HeatmapModel
from dataset import create_floor_step_loaders


def floor_step_loss_single(y, y_gt) -> Tensor:
    return torch.mean((y_gt - y) ** 2 * (y_gt * 0.9 + 0.1), dim=[1, 2, 3])


def floor_step_criterion(y, y_gt) -> Tensor:
    return torch.mean(floor_step_loss_single(y, y_gt))


def train_and_save_floor_step(channels, loaders=None, epochs=180):
    num_epochs = epochs
    # batch_size = 16
    learning_rate = 0.0003

    model = HeatmapModel(intermediate_channels=channels, output_channels=1, use_last_conv=True, use_inv_residual=True).cuda()

    num_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print("Number of trainable parameters: {}".format(num_params))
    summary(model, (3, 384, 288))

    # Loss and optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    if loaders is None:
        train_loader, test_loader = create_floor_step_loaders(16)
    else:
        train_loader, test_loader = loaders

    model.train()

    loss_history = []
    epoch_loss = float('inf')

    try:
        for epoch in range(num_epochs):
            loss_list = []

            t = tqdm(train_loader)
            for i, (images, labels) in enumerate(t):
                images = images.cuda()
                labels = labels.cuda()
                # Run the forward pass
                outputs = model(images)

                loss = floor_step_criterion(outputs, labels)
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

    except KeyboardInterrupt:
        print("Training stopped early by user")
        pass

    model_path = f"models/floor_door_edge_e{num_epochs}_c{channels}_loss{epoch_loss}_bn_inv_res.pt"
    torch.save({"model": model.state_dict(), "loss_history": loss_history}, model_path)

    plt.plot(loss_history, label='loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend(loc='upper right')
    plt.show()


if __name__ == "__main__":
    """
    c1_candidates = [8, 16, 32]
    c2_candidates = [16, 32, 48]
    c3_candidates = [32, 48, 64]
    c4_candidates = [48, 64, 96]
    c5_candidates = [64, 96, 128]
    c6_candidates = [96, 128, 192]

    input_channels_list = []

    # 4 channels
    for c1 in c1_candidates:
        for c2 in c2_candidates:
            for c3 in c3_candidates:
                for c4 in c4_candidates:
                    input_channels_list.append([c1, c2, c3, c4])

    # 5 channels
    for c1 in c1_candidates:
        for c2 in c2_candidates:
            for c3 in c3_candidates:
                for c4 in c4_candidates:
                    for c5 in c5_candidates:
                        input_channels_list.append([c1, c2, c3, c4, c5])

    # 3 channels
    for c1 in c1_candidates:
        for c2 in c2_candidates:
            for c3 in c3_candidates:
                input_channels_list.append([c1, c2, c3])

    # 6 channels
    for c1 in c1_candidates:
        for c2 in c2_candidates:
            for c3 in c3_candidates:
                for c4 in c4_candidates:
                    for c5 in c5_candidates:
                        for c6 in c6_candidates:
                            input_channels_list.append([c1, c2, c3, c4, c5, 6])
    """
    train_loader, test_loader = create_floor_step_loaders(batch_size=12)

    input_channels_list = [[24, 32, 64, 128, 128]]

    for channels in input_channels_list:
        train_and_save_floor_step(channels, (train_loader, test_loader), epochs=200)
