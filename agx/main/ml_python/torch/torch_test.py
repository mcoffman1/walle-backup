import torch
import torchvision


print("-----------------")
print(f"gpu is available = {torch.cuda.is_available()}")
print("-----------------")


tensor = torch.rand(3,4)
print(tensor)
print("-----------------")
print(f"matrix is on {tensor.device}")
print("-----------------")

device = "cuda" if torch.cuda.is_available() else "cpu"
tong = tensor.to(device)


print(tong)
print("-----------------")
print(f"matrix is on {tong.device}")
print("-----------------")

