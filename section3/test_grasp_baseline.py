import sys
sys.path.append(r'./thirdparty/graspnet-baseline')
from PIL import Image
import numpy as np
import scipy.io as scio
import os
from demo import demo_variable

data_dir = r'./thirdparty/graspnet-baseline/doc/example_data'
# 数据读取
rgb = np.array(Image.open(os.path.join(data_dir, 'color.png'))) 
depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
intrinsic = meta['intrinsic_matrix']

# 运行
demo_variable(rgb, depth, mask, intrinsic)