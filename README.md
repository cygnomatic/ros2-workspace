# VSCode ROS2 Workspace Template

This template will get you set up using ROS2 with VSCode as your IDE.

A fork of [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace) with better wsl GUI support.

## Usage

### Setup docker

#### Windows - WSL2

> **Warning**  
> 1. 请确保 **C 盘下至少有 5G 的空余空间**。否则可能安装失败。  
> 2. 由于网络原因，可能需要提前配置好系统代理。  

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

> **Tip**  
> Docker 服务会随 Docker Desktop 启动。启动完毕后可关闭 Docker Desktop，保留 Docker 服务在后台即可。

#### Linux

See [docker docs](https://docs.docker.com/engine/install/).

### Clone the repo

将本项目克隆至本地。并切换分支：
-  Windows: `wsl2` 分支
-  Linux x86_64: `main` 分支

### Open the repo in vscode

Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder).   

> **Warning**  
> 请在打开 DevContainer 时关闭代理软件的 TUN 模式。否则会导致 vscode 插件无法自动安装。

When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

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


## FAQ

### WSL2

#### The gui doesn't show up

请使用 `wsl2` 分支。使用 `main` 分支会导致无法使用 GUI。

#### I want to use vGPU

If you want to access the vGPU through WSL2, you'll need to add additional components to the `.devcontainer/devcontainer.json` file in accordance to [these directions](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)

```jsonc
	"runArgs": [
		"--network=host",
		"--cap-add=SYS_PTRACE",
		"--security-opt=seccomp:unconfined",
		"--security-opt=apparmor:unconfined",
		"--volume=/tmp/.X11-unix:/tmp/.X11-unix",
		"--volume=/mnt/wslg:/mnt/wslg",
		"--volume=/usr/lib/wsl:/usr/lib/wsl",
		"--device=/dev/dxg",
      		"--gpus=all"
	],
	"containerEnv": {
		"DISPLAY": "${localEnv:DISPLAY}", // Needed for GUI try ":0" for windows
		"WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}",
		"XDG_RUNTIME_DIR": "${localEnv:XDG_RUNTIME_DIR}",
		"PULSE_SERVER": "${localEnv:PULSE_SERVER}",
		"LD_LIBRARY_PATH": "/usr/lib/wsl/lib",
		"LIBGL_ALWAYS_SOFTWARE": "1" // Needed for software rendering of opengl
	},
```

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
