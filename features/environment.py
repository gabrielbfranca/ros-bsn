import subprocess
import time 
def before_all(context):
    
    context.bsn_launch = subprocess.Popen(
        ['ros2', 'launch', 'system', 'bsn_launch.py'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
    )
    time.sleep(15)

def after_all(context):
    #context.roscore.terminate()
    context.bsn_launch.terminate()