import subprocess
import time

# Specify the path to your Bash script
script_path = '/home/student/bash_files/test.sh'

# Execute the Bash script
subprocess.run(['bash', script_path], check=True)
num = 1
while True:
    num += 1
    print('running: ',num)
    time.sleep(1)
