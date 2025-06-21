import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fahad/bumperbot_ws/src/install/room_grid_navigator'
