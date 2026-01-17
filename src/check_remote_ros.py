import os
import pty
import time
import sys

def run_ssh_command(cmd, password):
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp(cmd[0], cmd)
    else:
        output = b""
        try:
            while True:
                try:
                    chunk = os.read(fd, 1024)
                except OSError:
                    break
                
                output += chunk
                
                if b"password:" in chunk.lower():
                    os.write(fd, (password + "\n").encode())
                
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

# Check running ROS 2 nodes on remote
print(f"Checking ROS2 nodes on {remote_ip}...")
# Try to source common ROS 2 locations.
remote_cmd = (
    "export ROS_DOMAIN_ID=12; "
    "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; "
    "elif [ -f /opt/ros/foxy/setup.bash ]; then source /opt/ros/foxy/setup.bash; "
    "elif [ -f /opt/ros/iron/setup.bash ]; then source /opt/ros/iron/setup.bash; "
    "elif [ -f /opt/ros/rolling/setup.bash ]; then source /opt/ros/rolling/setup.bash; "
    "elif [ -f /opt/ros/galactic/setup.bash ]; then source /opt/ros/galactic/setup.bash; "
    "fi; "
    "if [ -f ~/usv_workspace/install/setup.bash ]; then source ~/usv_workspace/install/setup.bash; fi; "
    "echo '--- Environment ---'; env | grep ROS; "
    "echo '--- Processes ---'; ps aux | grep usv | grep -v grep; "
    "echo '--- Nodes ---'; ros2 node list; "
    "echo '--- Topics ---'; ros2 topic list"
)

ssh_cmd = ["ssh", f"{user}@{remote_ip}", remote_cmd]
print(f"Running: {' '.join(ssh_cmd)}")
print("Output:")
print(run_ssh_command(ssh_cmd, password))

# Check environment of the running ROS process
print(f"Checking environment of usv_launch process on {remote_ip}...")
remote_cmd_env = (
    "pid=$(pgrep -f 'usv_launch.py' | head -n 1); "
    "if [ -z \"$pid\" ]; then "
    "   echo 'Process usv_launch.py not found'; "
    "else "
    "   echo \"Found PID: $pid\"; "
    "   echo '--- Environment Variables ---'; "
    "   cat /proc/$pid/environ | tr '\\0' '\\n' | grep FASTDDS || echo 'FASTDDS env var not found'; "
    "fi"
)

ssh_cmd_env = ["ssh", f"{user}@{remote_ip}", remote_cmd_env]
print(f"Running: {' '.join(ssh_cmd_env)}")
print("Output:")
print(run_ssh_command(ssh_cmd_env, password))

# Check environment of a specific child node process (e.g. usv_control_node)
print(f"Checking environment of a child node process on {remote_ip}...")
remote_cmd_child_env = (
    "pid=$(pgrep -f 'usv_control_node' | head -n 1); "
    "if [ -z \"$pid\" ]; then "
    "   echo 'Process usv_control_node not found'; "
    "else "
    "   echo \"Found PID: $pid\"; "
    "   echo '--- Environment Variables ---'; "
    "   cat /proc/$pid/environ | tr '\\0' '\\n' | grep FASTDDS || echo 'FASTDDS env var not found'; "
    "fi"
)

ssh_cmd_child_env = ["ssh", f"{user}@{remote_ip}", remote_cmd_child_env]
print(f"Running: {' '.join(ssh_cmd_child_env)}")
print("Output:")
print(run_ssh_command(ssh_cmd_child_env, password))

# Check RMW Implementation
print(f"Checking RMW Implementation on {remote_ip}...")
remote_cmd_rmw = (
    "if [ -f /opt/ros/kilted/setup.bash ]; then source /opt/ros/kilted/setup.bash; fi; "
    "echo '--- RMW Info ---'; "
    "env | grep RMW; "
    "ros2 doctor --report | grep -i middleware || echo 'ros2 doctor not available or failed'"
)

ssh_cmd_rmw = ["ssh", f"{user}@{remote_ip}", remote_cmd_rmw]
print(f"Running: {' '.join(ssh_cmd_rmw)}")
print("Output:")
print(run_ssh_command(ssh_cmd_rmw, password))

# Check Ping from Remote to Local
print(f"Checking Ping from {remote_ip} to Ground Station (192.168.68.53)...")
remote_cmd_ping = "ping -c 4 192.168.68.53"
ssh_cmd_ping = ["ssh", f"{user}@{remote_ip}", remote_cmd_ping]
print(f"Running: {' '.join(ssh_cmd_ping)}")
print("Output:")
print(run_ssh_command(ssh_cmd_ping, password))

# Check iptables on Remote (requires sudo usually, but let's try)
print(f"Checking iptables on {remote_ip}...")
remote_cmd_fw = "sudo iptables -L -n -v | head -n 20"
ssh_cmd_fw = ["ssh", f"{user}@{remote_ip}", remote_cmd_fw]
print(f"Running: {' '.join(ssh_cmd_fw)}")
print("Output:")
print(run_ssh_command(ssh_cmd_fw, password))
