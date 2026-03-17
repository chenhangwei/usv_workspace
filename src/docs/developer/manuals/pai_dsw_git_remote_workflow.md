# PAI-DSW Git And Remote Workflow

This manual describes the recommended way to move the current ROS 2 + RL workspace to Alibaba Cloud PAI-DSW, keep the code in Git, and continue tuning from VS Code.

## Recommended Setup

Use Git for source code only.

Do not commit these categories of files:

- Python virtual environments
- `build/`, `install/`, `log/`
- `tmp_rl/` and `src/tmp_rl/`
- model checkpoints such as `.pt`, `.pth`, `.npz`, `.ckpt`, `.onnx`
- ROS bag and recorded runtime data

Store training outputs on mounted cloud storage or a dedicated data directory, not inside the Git history.

## Environment Recommendation

For this workspace, the cloud image should match the current local training stack as closely as possible.

Recommended baseline:

- Ubuntu 24.04 compatible environment
- ROS 2 Jazzy
- Python 3.12
- `git`, `colcon`, `rosdep`
- all package dependencies required by the ROS 2 workspace

If the default PAI-DSW image does not provide ROS 2 Jazzy, use a custom image. This project is not a pure Python training repo; the RL loop depends on ROS 2 workspace components.

## Local Preparation

From the local workspace root, verify that only source and configuration files remain visible to Git:

```bash
cd ~/usv_workspace
git status --short
```

After the top-level ignore rules are in place, `tmp_rl/`, `.venv/`, and ROS build artifacts should no longer appear as untracked content.

Then push the repository to your Git hosting provider:

```bash
cd ~/usv_workspace
git remote add origin <your-git-remote>
git add .
git commit -m "Prepare workspace for cloud training"
git push -u origin main
```

If the remote already exists, only run the last three commands.

If you use Git over SSH, the remote usually looks like one of these:

```bash
git remote add origin git@github.com:<owner>/<repo>.git
```

```bash
git remote add origin git@gitlab.com:<owner>/<repo>.git
```

```bash
git remote add origin git@codeup.aliyun.com:<group>/<repo>.git
```

Before the first push, make sure your local public key has been added to the Git hosting account.

## Clone On PAI-DSW

In the PAI-DSW terminal:

```bash
git clone <your-git-remote>
cd usv_workspace
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If the cloud environment does not allow `rosdep install`, install the missing dependencies in the custom image ahead of time.

## VS Code Remote Connection

There are two practical options.

### Option A: Remote-SSH

Use this when the PAI-DSW instance exposes SSH direct access.

This follows the official PAI-DSW SSH direct-connect model.

Applicable instance scope:

- pay-as-you-go DSW instances created from a public resource group, except instance types starting with `ecs.ebm`
- DSW instances created from Lingjun compute resources

Requirements:

- SSH enabled in the DSW instance configuration
- your public key added to the DSW SSH configuration
- SSH server installed and running inside the instance
- VPC, vSwitch, and security group configured in the same region as the instance
- inbound TCP port `22` allowed by the security group for VPC access
- if public access is enabled, a public NAT gateway and EIP configured for the instance

Then connect from VS Code using Remote-SSH and open the cloned `usv_workspace` directory.

For Alibaba Cloud PAI-DSW, do not assume the raw VPC private IP can be used directly from your local machine.
The supported SSH entry points are the DSW public access endpoint or the DSW VPC access endpoint shown in the instance network settings.

When public access is enabled, PAI automatically creates the required DNAT rule on the selected public NAT gateway. You do not need to create the DNAT rule manually.

Typical SSH entry modes are:

### Public Access With EIP

Use the EIP and the configured public access port from the DSW instance network settings:

```bash
ssh -i ~/.ssh/id_rsa root@<public-eip> -p <public-port>
```

`<public-eip>` is the instance EIP.

`<public-port>` is the public access port configured for the instance.

### VPC Access With DSW Domain

If you are connecting from another machine inside the same VPC path, such as an ECS instance, use the DSW VPC domain shown in the instance network settings:

```bash
ssh -i ~/.ssh/id_rsa root@<dsw-domain> -p 22
```

Example form:

```bash
ssh -i ~/.ssh/id_rsa root@dsw-notebook-xxxx.dsw-xxxx.dsw.pai.alibaba.com -p 22
```

Officially, VPC access is intended for another terminal inside the VPC, for example an ECS instance.

Before using the VPC domain, make sure private DNS resolution is enabled for the VPC. The DSW VPC endpoint relies on private DNS to resolve the domain name to the instance private IP.

Do not use addresses like `10.x.x.x` or `192.168.x.x` directly from your local laptop unless that machine already has a routed path into the target VPC.

### Remote-SSH Config Example

Add a host entry to the local `~/.ssh/config`:

```ssh-config
Host pai-dsw
	HostName <public-eip-or-dsw-domain>
	User root
	Port <public-port-or-22>
	IdentityFile ~/.ssh/id_rsa
	ServerAliveInterval 30
	ServerAliveCountMax 120
```

Replace `root` with the actual login user if your instance uses a non-root account.

For public access, set `HostName` to the EIP and `Port` to the configured public access port.

For VPC access, set `HostName` to the DSW domain and keep `Port 22`.

Then test from a local terminal:

```bash
ssh pai-dsw
```

If that succeeds, open VS Code and use Remote-SSH to connect to `pai-dsw`, then open the cloud-side `usv_workspace` folder.

### Public Key Placement

If the cloud instance is a standard Linux VM behind PAI-DSW, add your local public key to the remote account:

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
echo "<your-local-public-key>" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
```

If the platform already provides a console for binding SSH keys, use that mechanism instead of manually editing `authorized_keys`.

### Custom Image Note

If the DSW instance uses an official preset image, or a custom image derived from an official preset image, SSH server is usually already installed.

If you use another custom image, install and start SSH server inside the instance:

```bash
sudo apt-get update
sudo apt-get install openssh-server
sudo service ssh start
service ssh status
```

If `sudo` is missing in the image, install it first or run the equivalent commands as `root`.

### DSW Access Notes

- DSW SSH login uses the `root` account by default
- when opening from VS Code, the initial directory may be `/root`; open `/mnt/workspace` manually to access your project files
- if you want both VPC access and public access, add all needed client public keys to the DSW SSH configuration

### Option B: No SSH Available

If PAI-DSW does not expose SSH, use one of these alternatives:

- VS Code Tunnel if the instance allows it
- the PAI-DSW web IDE or terminal
- Jupyter terminal plus Git sync

This still supports Git-based collaboration, but it is not the same as full Remote-SSH.

## Build And Training Loop In Cloud

After connecting to the cloud workspace, the minimal loop is:

```bash
cd ~/usv_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select usv_rl --symlink-install --allow-overriding usv_rl
source install/setup.bash
```

Then run the RL training or evaluation commands from the workspace root.

Keep long-running experiment outputs outside Git, for example:

```bash
mkdir -p ~/cloud_runs/usv_rl
```

and point training outputs there if you want the artifacts to survive code resets or branch switching.

## Recommended DSW Directory Layout

For this repository, keep source code, training artifacts, and reusable datasets separated.

Recommended layout inside DSW:

```text
/mnt/workspace/
	usv_workspace/              # Git clone of the source repository

/mnt/data/
	datasets/                   # reusable teacher datasets and curated inputs
	checkpoints/                # .pt, .npz, checkpoint directories
	evals/                      # ranking json, validation json, summary json
	logs/                       # text logs, terminal captures, experiment notes
```

Recommended meanings:

- `/mnt/workspace/usv_workspace`: only source code and developer docs managed by Git
- `/mnt/data/datasets`: data assets that should survive branch switching and rebuilds
- `/mnt/data/checkpoints`: all large training outputs that must never enter Git
- `/mnt/data/evals`: evaluation outputs worth keeping across retraining rounds
- `/mnt/data/logs`: optional persistent logs for later debugging

Create the layout once after mounting NAS:

```bash
mkdir -p /mnt/workspace/usv_workspace
mkdir -p /mnt/data/datasets
mkdir -p /mnt/data/checkpoints
mkdir -p /mnt/data/evals
mkdir -p /mnt/data/logs
```

## Path Conventions For This Project

Use these path conventions when moving the current local commands to DSW.

### Git Clone

```bash
cd /mnt/workspace
git clone <your-git-remote> usv_workspace
cd /mnt/workspace/usv_workspace
```

### Build

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select usv_rl --symlink-install --allow-overriding usv_rl
source install/setup.bash
```

### Training Output Example

Instead of writing into `./src/tmp_rl`, write into `/mnt/data/checkpoints` and `/mnt/data/evals`:

```bash
cd /mnt/workspace/usv_workspace
source install/setup.bash
/bin/python3 -m usv_rl.train_mappo_residual \
	--output /mnt/data/checkpoints/mappo_balanced_repair_selective_t1280_ckpt.pt \
	--checkpoint-dir /mnt/data/checkpoints/mappo_balanced_repair_selective_t1280_ckpts \
	--checkpoint-ranking-json /mnt/data/evals/mappo_balanced_repair_selective_t1280_ranking.json \
	--checkpoint-eval-json-dir /mnt/data/evals/mappo_balanced_repair_selective_t1280_eval
```

### Validation Output Example

```bash
cd /mnt/workspace/usv_workspace
source install/setup.bash
/bin/python3 -m usv_rl.evaluate_mappo_residual \
	--model /mnt/data/checkpoints/mappo_balanced_repair_selective_t1280_ckpts/mappo_balanced_repair_selective_t1280_ckpt_step_0000320.pt \
	--episodes 15 \
	--steps-per-episode 180 \
	--scenario five_usv_dense_head_on \
	--scenario five_usv_dense_crossing \
	--scenario five_usv_dense_overtaking \
	--output-json /mnt/data/evals/mappo_balanced_repair_selective_step320_180_ep15.json \
	--device cpu
```

### Dataset Placement Example

Scripted teacher datasets and curated training inputs should go under:

```text
/mnt/data/datasets/
```

For example:

```text
/mnt/data/datasets/bc_scripted_teacher_mlp_v2_dense_dataset.npz
```

## Suggested Git Strategy

Recommended branch usage:

- `main`: stable runnable source
- `exp/<topic>`: reward tuning, curriculum changes, evaluator fixes

Commit source changes often, but do not commit checkpoints.

For experiment traceability, commit:

- training commands
- reward or curriculum parameter changes
- small JSON summaries if they are important and intentionally curated

Do not commit:

- raw logs
- checkpoint directories
- generated datasets unless they are intentionally versioned data assets

## First Cloud Validation Checklist

Before starting a long PAI-DSW run, verify:

1. `git status` is clean after clone.
2. `source /opt/ros/jazzy/setup.bash` succeeds.
3. `colcon build --packages-select usv_rl --symlink-install --allow-overriding usv_rl` succeeds.
4. `source install/setup.bash` succeeds.
5. a short evaluator or smoke command can run in the cloud environment.
6. in the DSW network settings, the SSH section shows either a valid public access endpoint or a valid VPC access endpoint.
7. for public access, confirm the selected NAT gateway, EIP, and public access port are present.
8. for VPC access, confirm security group port `22` is open and private DNS is enabled for the VPC.

Once these checks pass, the workspace is ready for cloud-side code changes, retraining, and checkpoint evaluation.