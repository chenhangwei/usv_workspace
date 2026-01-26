#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of system command handler.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
System Command Handler for Remote USV Management
Handles SSH command execution including password-protected sudo operations.
"""

import subprocess
import logging

class SystemCommandHandler:
    def __init__(self):
        self.logger = logging.getLogger('SystemCommandHandler')

    def execute_ssh_command(self, hostname, username, command, password=None, timeout=10):
        """
        Execute a command via SSH on remote host.
        If password is provided, it can be used for authentication if configured (though auto-ssh is preferred).
        This method mainly wraps the subprocess call.
        """
        ssh_cmd = f"ssh -o ConnectTimeout={timeout} -o StrictHostKeyChecking=no {username}@{hostname} '{command}'"
        
        try:
            # If we need to pass password for sudo inside the command, the command string should already contain it.
            # But if we need to pass password for SSH login itself, we might need sshpass (but standard is key-based).
            # Here we assume key-based auth for SSH, and password is used for sudo loop inside the shell.
            
            result = subprocess.run(
                ssh_cmd, 
                shell=True, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                timeout=timeout
            )
            
            if result.returncode == 0:
                self.logger.info(f"Command successful on {hostname}: {command}")
                return True, result.stdout.decode('utf-8')
            else:
                error_msg = result.stderr.decode('utf-8')
                self.logger.error(f"Command failed on {hostname}: {error_msg}")
                return False, error_msg
                
        except subprocess.TimeoutExpired:
            self.logger.error(f"Command timeout on {hostname}")
            return False, "Timeout"
        except Exception as e:
            self.logger.error(f"Exception executing command on {hostname}: {e}")
            return False, str(e)

    def reboot_usv(self, hostname, username, password=None):
        """
        Reboot the remote USV.
        If password is provided, uses 'echo password | sudo -S reboot'.
        Otherwise uses 'sudo -n reboot'.
        """
        if password:
            # Use -S to read password from stdin
            # Note: We wrap in sh -c to handle the pipe properly over SSH
            remote_cmd = f"echo '{password}' | sudo -S reboot"
        else:
            remote_cmd = "sudo -n reboot"
            
        return self.execute_ssh_command(hostname, username, remote_cmd)
