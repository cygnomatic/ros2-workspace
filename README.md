# VSCode ROS2 Workspace Template

This template will get you set up using ROS2 with VSCode as your IDE.

A fork of [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace) with better wsl2 support.

## Setup Tutorial

### Prerequisite

1. **系统盘下至少有 5G 的空余空间**。否则可能安装失败。  
2. 由于网络原因，可能需要提前配置好系统代理。  
3. 设备有可用的 Nvidia 显卡。

### Setup docker

#### Windows - WSL2

1. 使用管理员权限打开终端

2. 在打开的终端中运行：
	```powershell
	wsl --install
	```

3. 安装 Docker Desktop。

	```powershell
	winget install Docker.DockerDesktop
	```

4. 打开 Docker Desktop，跳过登录和那些麻烦的选择。

> [!TIP]  
> Docker 服务会随 Docker Desktop 启动。启动完毕后可关闭 Docker Desktop，保留 Docker 服务在后台即可。

#### Linux

See [docker docs](https://docs.docker.com/engine/install/).

### Clone the repo

将本项目克隆至本地。并切换分支：
-  Windows: `wsl2` 分支
-  Linux x86_64: `main` 分支

### Open the repo in vscode

1. 使用 vscode 打开本项目目录   

   Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder).   

2. 安装 vscode 插件 `Dev Container`

   按下快捷键 `Ctrl+P` 调出 Vscode Quick Open，输入：
   ```
   ext install ms-vscode-remote.remote-containers
   ```

3. 启动 DevContainer

   按下快捷键 `Ctrl+Shift+P`，输入 `rebuild`，选择 `Dev Containers: Rebuild and Reopen in Container`

> [!NOTE]  
> 请在打开 DevContainer 时关闭代理软件的 TUN 模式。否则会导致 vscode 插件无法自动安装。

4. 等待启动。第一次启动需要构筑镜像，可能会花费长达 10 分钟。

If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

### Update the template with your code

1. Specify the repositories you want to include in your workspace in `src/ros2.repos` or delete `src/ros2.repos` and develop directly within the workspace.
2. If you are using a `ros2.repos` file, import the contents `Terminal->Run Task..->import from workspace file`
3. Install dependencies `Terminal->Run Task..->install dependencies`
4. (optional) Adjust scripts to your liking.  These scripts are used both within tasks and CI.
   * `setup.sh` The setup commands for your code.  Default to import workspace and install dependencies.
   * `build.sh` The build commands for your code.  Default to `--merge-install` and `--symlink-install`
   * `test.sh` The test commands for your code.
5. Develop!

## Optional Setup

### Configurate USB passthrough (WSL Only)

若你需要在开发容器中使用 USB 设备（如工业相机、串口等），请进行以下操作。
Linux 系统不需要按照此部分操作。可直接使用 USB 设备。

> [!NOTE]  
> 需要更好的解决方案。Docker Desktop 运行于一个基于 Alpine 的 WSL 镜像 `docker-desktop`，而 usbipd-win 不支持直接 attach 到 `docker-desktop` 内。[此 issue](https://github.com/dorssel/usbipd-win/issues/669) 建议安装一个单独的 Ubuntu 22.04 镜像用于接入 USB。能用但不优雅。

1. 下载 usbipd-win >= 4.0
   
   ```
	winget install usbipd
   ```

> [!IMPORTANT]   
> 请确保你的 `usbipd-win` 版本 >= 4.0. 你可以手动从 [GitHub Release](https://github.com/dorssel/usbipd-win/releases) 安装最新版。


2. 安装 Ubuntu-22.04 WSL
   
   ```powershell
   wsl --install Ubuntu-22.04
   ```
   首次运行请等待安装完成。安装完成后会自动切入 WSL Ubuntu-22.04 系统内终端。  
   **保持此窗口在后台运行，直到完成此部分前不要关闭。**

3. 列出设备的 BUSID  
   
   启动一个有管理员权限的新终端，运行：
   ```powershell
   usbipd list
   ```
   输出应当类似于：
   ```
   Connected:
	BUSID  VID:PID    DEVICE                     STATE
	1-3    27c6:589a  Goodix fingerprint         Not shared
	2-3    8087:0032  英特尔(R) 无线 Bluetooth(R)  Not shared
	3-1    2b7e:b557  XiaoMi USB 2.0 Webcam      Not shared
   
   Persisted:
   	GUID                                  DEVICE
   ```
   记住你期望在 Docker 中使用的设备的 BUSID。

4. 设置默认 WSL Distro
   ```powershell
   wsl --set-default Ubuntu-22.04
   ```

5. 配置待穿透设备
   ```powershell
   usbipd bind --busid=<BUSID>
   usbipd attach --wsl --busid=<BUSID>
   ```
   **将你需要在 Docker 环境中使用的设备 BUSID 替换掉命令中的 `<BUSID>`。**  
   例如在上述 `usbipd list` 的输出示例中，你希望将 XiaoMi USB 2.0 Webcam 映射到 Docker 中：
   ```powershell
   usbipd bind --busid=3-1
   ```

> [!TIP]  
> 每次重启宿主机，你需要重新运行第 5 步。


### Setup CUDA

Docker 镜像中已内置 CUDA。不需要进行任何手动配置。  
无需安装 CUDA for WSL2 或在 WSL 中安装 CUDA。  

## FAQ

### WSL2

#### The gui doesn't show up

请使用 `wsl2` 分支。使用 `main` 分支会导致无法使用 GUI。

### Repos are not showing up in VS Code source control

This is likely because vscode doesn't necessarily know about other repositories unless you've added them directly. 

```
File->Add Folder To Workspace
```

![Screenshot-26](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/d8711320-2c16-463b-9d67-5bd9314acc7f)


Or you've added them as a git submodule.

![Screenshot-27](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/8ebc9aac-9d70-4b53-aa52-9b5b108dc935)

To add all of the repos in your *.repos file, run the script

```bash
python3 .devcontainer/repos_to_submodules.py
```

or run the task titled `add submodules from .repos`
