### These are the steps I had to follow to get Jetson Inference working on my AGX Orin

# Jetson Inference Setup Guide

This guide helps you get up and running with [dusty-nv/jetson-inference](https://github.com/dusty-nv/jetson-inference) on your Jetson device (e.g., AGX Xavier, Orin).

---

## **Step 1: Clone the Repository**
Use the `--recursive` flag to ensure all submodules are downloaded:

```bash
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference.git
cd jetson-inference
```

---

## **Step 2: Install Docker**

```bash
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable docker
sudo systemctl start docker
```

Verify installation:
```bash
docker --version
```

---

## **Step 3: Install NVIDIA Container Toolkit**

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add - \
    && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Configure Docker to Use NVIDIA Runtime
Create or edit `/etc/docker/daemon.json`:

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```

Restart Docker:
```bash
sudo systemctl restart docker
```

Check runtime:
```bash
sudo docker info | grep -i runtime
```

Expected output includes:
```
Runtimes: runc nvidia io.containerd.runc.v2
Default Runtime: nvidia
```

---

## **Step 4: Run the Container**

Once everything is installed and configured, run the container:

```bash
./docker/run.sh
```

---

## **Troubleshooting**

- If you get a `permission denied` error on Docker commands:
  ```bash
  sudo usermod -aG docker $USER
  newgrp docker
  ```

- If Docker says `unknown or invalid runtime name: nvidia`, double check your installation and `/etc/docker/daemon.json` file.

---

## ✅ You're all set!

You should now have Jetson Inference running in a container on your Jetson device. Happy coding! 🚀

