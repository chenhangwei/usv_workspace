import os
import pty
import time

def run_ssh_command(cmd, password):
    pid, fd = pty.fork()
    if pid == 0:
        # Child process: exec the command
        os.execvp(cmd[0], cmd)
    else:
        # Parent process: handle IO
        output = b""
        try:
            while True:
                try:
                    chunk = os.read(fd, 1024)
                except OSError:
                    break
                
                output += chunk
                
                # Check for password prompt
                if b"password:" in chunk.lower():
                    os.write(fd, (password + "\n").encode())
                
                # Check for "Are you sure..." (host key verification)
                if b"continue connecting" in chunk.lower() or b"fingerprint" in chunk.lower():
                    os.write(fd, b"yes\n")
                    
        except Exception:
            pass
        finally:
            os.close(fd)
            os.waitpid(pid, 0)
        
        return output.decode('utf-8', errors='ignore')

remote_ip = "192.168.68.54"
user = "chenhangwei"
password = "c3467684"
remote_path = "/home/chenhangwei/fastdds_usv.xml"
local_temp_file = "temp_fastdds_usv.xml"

xml_content = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="usv_unicast_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                </discovery_config>
                <initialPeersList>
                    <!-- Point to GS IP -->
                    <locator>
                        <udpv4>
                            <address>192.168.68.53</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
"""

# Write content to local file first
with open(local_temp_file, "w") as f:
    f.write(xml_content)

print(f"Transferring config to {remote_ip}...")

# Use scp to transfer
scp_cmd = ["scp", "-o", "StrictHostKeyChecking=no", local_temp_file, f"{user}@{remote_ip}:{remote_path}"]
output = run_ssh_command(scp_cmd, password)
# print("SCP Output:", output)

# Verify
print("Verifying remote file...")
ssh_cmd = ["ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{remote_ip}", f"cat {remote_path}"]
remote_content = run_ssh_command(ssh_cmd, password)

if "usv_unicast_profile" in remote_content and "192.168.68.53" in remote_content:
    print("SUCCESS: Remote XML updated successfully!")
else:
    print("WARNING: Remote file verification failed!")
    print("Remote content excerpt:", remote_content[:100])

# Clean up
if os.path.exists(local_temp_file):
    os.remove(local_temp_file)

# 1. Kill existing usv_launch process and all ROS nodes
print("Killing existing usv_launch process and ALL ros/python processes related to USV...")
# aggressive kill sequence
cmd_kill = (
    "pkill -f usv_launch.py; "
    "pkill -f ros2; "
    "pkill -f usv_bringup; "
    "pkill -f usv_comm; "
    "pkill -f usv_control; "
    "pkill -f usv_drivers; "
    "pkill -f usv_fan; "
    "pkill -f usv_led; "
    "pkill -f usv_sound; "
    "pkill -f usv_action; "
    "pkill -f mavros; "
    "sleep 2; "
    "pkill -9 -f usv_launch.py; " # Force kill if still alive
    "echo 'Clean up signals sent.'"
)
ssh_cmd_kill = ["ssh", f"{user}@{remote_ip}", cmd_kill]
print(run_ssh_command(ssh_cmd_kill, password))

time.sleep(3)

# 2. Restart usv_launch with FASTDDS_DEFAULT_PROFILES_FILE env var
print("Restarting usv_launch with FastDDS XML...")
# Using nohup to keep it running after ssh disconnects
# Sourcing the environment before running
remote_start_cmd = (
    "export FASTDDS_DEFAULT_PROFILES_FILE=/home/chenhangwei/fastdds_usv.xml; "
    "export ROS_DOMAIN_ID=12; "
    "if [ -f /opt/ros/kilted/setup.bash ]; then source /opt/ros/kilted/setup.bash; fi; "
    "if [ -f ~/usv_workspace/install/setup.bash ]; then source ~/usv_workspace/install/setup.bash; fi; "
    "nohup ros2 launch usv_bringup usv_launch.py > /dev/null 2>&1 &"
)
ssh_cmd_start = ["ssh", f"{user}@{remote_ip}", remote_start_cmd] 
print(f"Running: {' '.join(ssh_cmd_start)}")
print(run_ssh_command(ssh_cmd_start, password))

print("USV launch restarted (hopefully). Verify with check_remote_ros.py in a few seconds.")
