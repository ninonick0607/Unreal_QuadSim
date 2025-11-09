import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ninonick0426/Documents/GitHub/Unreal_QuadSim/Plugins/QuadSimPlugin/Resnet/install/adaptive_resnet_controller'
