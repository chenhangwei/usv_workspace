import paramiko
import sys

hostname = "192.168.68.54"
username = "chenhangwei"
password = "c3467684"
remote_file_path = "/home/chenhangwei/fastdds_usv.xml"

xml_content = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="usv_unicast_profile" is_default_profile="true">
        <rtps>
            <transport_descriptors>
                <transport_id>udp_transport</transport_id>
            </transport_descriptors>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <ignoreParticipantFlags>FILTER_DIFFERENT_HOST</ignoreParticipantFlags>
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
</profiles>"""

try:
    print(f"Connecting to {hostname}...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname, username=username, password=password)
    
    print("Writing corrected XML file via SFTP...")
    sftp = ssh.open_sftp()
    with sftp.file(remote_file_path, 'w') as f:
        f.write(xml_content)
    sftp.close()
    
    print("Verifying file content on remote server...")
    stdin, stdout, stderr = ssh.exec_command(f'cat {remote_file_path}')
    remote_content = stdout.read().decode()
    if "gs_unicast_profile" not in remote_content and "usv_unicast_profile" in remote_content:
        print("Verification SUCCESS: File updated correctly.")
    else:
        print("Verification WARNING: File content might not be as expected.")
        # print(remote_content)

    print("Checking if any other files need cleanup...")
    # Optional: check if there are other files
    
    ssh.close()
    print("Done.")

except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
