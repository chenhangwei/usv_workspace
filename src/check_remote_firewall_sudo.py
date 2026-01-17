import os
import pty
import time
import sys

def run_ssh_sudo_command(cmd, password, sudo_password):
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
                
                # SSH password prompt
                if b"password:" in chunk.lower() and b"sudo" not in chunk.lower() :
                     # Simple heuristic: if it looks like SSH password prompt
                     if b"chenhangwei@" in chunk or b"'s password:" in chunk:
                        os.write(fd, (password + "\n").encode())
                
                # Sudo password prompt
                if b"[sudo] password for" in chunk.lower() or b"sudo: a password is required" in chunk:
                    os.write(fd, (sudo_password + "\n").encode())

                # Continue prompt
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

# We use -t to force pseudo-tty allocation so sudo will prompt
commands = [
    ("Checking Firewall Status (ufw)", ["ssh", "-o", "StrictHostKeyChecking=no", "-t", f"{user}@{remote_ip}", "sudo ufw status"]),
]

for title, cmd in commands:
    print(f"\n=== {title} ===")
    output = run_ssh_sudo_command(cmd, password, password)
    print(output)
