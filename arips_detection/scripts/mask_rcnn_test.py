import torchvision
import torch
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import cv2
import matplotlib.pyplot as plt
from torch.utils.cpp_extension import CUDA_HOME

print(CUDA_HOME)

# load a model pre-trained pre-trained on COCO
model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()
model = model.cuda()

img = cv2.imread('images/horse.jpg')
assert img is not None
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

img_tensor = torch.from_numpy(img).permute(2, 0, 1) / 255.0

result = model([img_tensor])[0]

print(result)

boxes = result['boxes'].detach().numpy().round()

for box in boxes:
    cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (255, 0, 0))

plt.imshow(img)
plt.show()
