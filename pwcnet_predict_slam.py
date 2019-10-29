from __future__ import absolute_import, division, print_function
from copy import deepcopy
from tfoptflow.tfoptflow.model_pwcnet import ModelPWCNet, _DEFAULT_PWCNET_TEST_OPTIONS
import skimage.io
import numpy as np
from tfoptflow.tfoptflow.visualize import display_img_pairs_w_flows

# Here, we're using a GPU (use '/device:CPU:0' to run inference on the CPU)
gpu_devices = ['/device:GPU:0']
controller = '/device:GPU:0'

# More options...
# Set the number of samples to visually inspect after inference
# Set the path to the trained model (make sure you've downloaded it first from http://bit.ly/tfoptflow)
# Set the batch size
ckpt_path = './models/pwcnet-lg-6-2-multisteps-chairsthingsmix/pwcnet.ckpt-595000'
batch_size = 1

# Configure the model for inference, starting with the default options
nn_opts = deepcopy(_DEFAULT_PWCNET_TEST_OPTIONS)
nn_opts['verbose'] = True
nn_opts['ckpt_path'] = ckpt_path
nn_opts['batch_size'] = batch_size
nn_opts['use_tf_data'] = True
nn_opts['gpu_devices'] = gpu_devices
nn_opts['controller'] = controller

# We're running the PWC-Net-large model in quarter-resolution mode
# That is, with a 6 level pyramid, and upsampling of level 2 by 4 in each dimension as the final flow prediction
nn_opts['use_dense_cx'] = True
nn_opts['use_res_cx'] = True
nn_opts['pyr_lvls'] = 6
nn_opts['flow_pred_lvl'] = 2

# The size of the images in this dataset are not multiples of 64, while the model generates flows padded to multiples
# of 64. Hence, we need to crop the predicted flows to their original size
nn_opts['adapt_info'] = (1, 436, 1024, 2)

# Instantiate the model in inference mode and display the model configuration
nn = ModelPWCNet(mode='test', options=nn_opts, dataset=None)
nn.print_config()

# im1 = skimage.io.imread("/home/cs4li/Dev/TUMVIO/dataset-slides2_512_16/mav0/cam0/data/1520520806923245321.png")
# im2 = skimage.io.imread("/home/cs4li/Dev/TUMVIO/dataset-slides2_512_16/mav0/cam0/data/1520520807073249321.png")

# im1 = skimage.io.imread("/home/cs4li/Dev/TUMVIO/dataset-slides2_512_16/mav0/cam0/data/1520520841224435391.png")
# im2 = skimage.io.imread("/home/cs4li/Dev/TUMVIO/dataset-slides2_512_16/mav0/cam0/data/1520520841374393391.png")

im1 = skimage.io.imread("/home/cs4li/Dev/TUMVIO/dataset-slides2_512_16/mav0/cam0/data/1520520934277953303.png")
im2 = skimage.io.imread("/home/cs4li/Dev/TUMVIO/dataset-slides2_512_16/mav0/cam0/data/1520520934427957303.png")

im1 = np.stack([im1, im1, im1], axis=-1).astype(np.float32) * 2 ** 8 / 2 ** 16
im2 = np.stack([im2, im2, im2], axis=-1).astype(np.float32) * 2 ** 8 / 2 ** 16
ret = nn.predict_from_img_pairs(((im1, im2,),), batch_size=1, verbose=True)

display_img_pairs_w_flows([np.stack([im1, im2, ]) / 255.], ret)

print("Done!")
