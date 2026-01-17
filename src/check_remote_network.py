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

commands = [
    ("Checking Ping to GS", ["ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{remote_ip}", "ping -c 3 192.168.68.53"]),
    ("Checking Firewall Status (ufw)", ["ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{remote_ip}", "sudo ufw status"]),
    ("Checking IP Tables (Input Chain)", ["ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{remote_ip}", "sudo iptables -L INPUT -n"])
]

for title, cmd in commands:
    print(f"\n=== {title} ===")
    output = run_ssh_command(cmd, password)
    print(output)
