import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rahul1/Desktop/UMD/Fall 24/Robot Modeling/Projects/Project0/testWS/src/tb_control/install/tb_control'
