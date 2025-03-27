import subprocess

# Call the Bash script to launch the rospy node
#subprocess.run(['bash', 'launch_script.sh'])

# Retrieve the PID of the rospy node
output = subprocess.check_output(['pgrep', '-f', 'test.py'])
pids = output.decode().split()  # Convert bytes to string and remove trailing newline
pid = pids[0] if pids else None

# Terminate the rospy node
subprocess.run(['kill', pid])

print(pid)



# Retrieve the PID of the rospy node
#output = subprocess.check_output(['pgrep', '-f', 'test.py'])
#pids = output.decode().split()  # Split the output by whitespace
#pid = pids[0] if pids else None  # Get the first PID or None if no PIDs found


