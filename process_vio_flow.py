from __future__ import absolute_import, division, print_function
from copy import deepcopy
from tfoptflow.tfoptflow.model_pwcnet import ModelPWCNet, _DEFAULT_PWCNET_TEST_OPTIONS
import skimage.io
import numpy as np
import os
import natsort
import argparse
import time


class ComputeFlow(object):
    def __init__(self):
        # Here, we're using a GPU (use '/device:CPU:0' to run inference on the CPU)
        gpu_devices = ['/device:GPU:0']
        controller = '/device:GPU:0'

        # More options...
        # Set the number of samples to visually inspect after inference
        # Set the path to the trained model (make sure you've downloaded it first from http://bit.ly/tfoptflow)
        # Set the batch size
        ckpt_path = './models/pwcnet-lg-6-2-multisteps-chairsthingsmix/pwcnet.ckpt-595000'
        # ckpt_path = './models/pwcnet-sm-6-2-multisteps-chairsthingsmix/pwcnet.ckpt-592000'
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
        self.nn = ModelPWCNet(mode='test', options=nn_opts, dataset=None)
        self.nn.print_config()

    def compute_nn_full_flow(self, im1, im2):
        im1 = np.stack([im1, im1, im1], axis=-1)
        im2 = np.stack([im2, im2, im2], axis=-1)
        return self.nn.predict_from_img_pairs(((im1, im2,),), batch_size=1, verbose=False)

    def compute_nn_flow(self, im1, im2, cur_pts):
        ret = self.compute_nn_full_flow(im1, im2)
        flow_sampled = ret[0][cur_pts.astype(np.uint)[:, 1], cur_pts.astype(np.uint)[:, 0], :]

        return cur_pts + flow_sampled, ret[0]


def main():
    flow_computer = ComputeFlow()
    arg_parser = argparse.ArgumentParser(description="Preprocess VIO Flow")
    arg_parser.add_argument("dataset", type=str, choices=["EUROC", "TUMVIO", "UZH-FPV"])
    arg_parser.add_argument("path", type=str)
    arg_parsed = arg_parser.parse_args()

    dataset = arg_parsed.dataset
    seq_dir = arg_parsed.path

    images = []
    if dataset == "TUMVIO":
        images = os.listdir(os.path.join(seq_dir, "mav0", "cam0", "data"))
        images = natsort.natsorted(images)
        timestamps = [int(i.split(".")[0]) for i in images]

        # add full path
        images = [os.path.join(seq_dir, "mav0", "cam0", "data", i) for i in images]
    elif dataset == "UZH-FPV":
        raise NotImplementedError("Not implemented")
    elif dataset == "EUROC":
        raise NotImplementedError("Not implemented")

    prev_image = skimage.img_as_ubyte(skimage.io.imread(images[0]))
    for i in range(1, len(images)):
        curr_img = skimage.img_as_ubyte(skimage.io.imread(images[i]))
        start_time = time.time()
        flow_computer.compute_nn_full_flow(prev_image, curr_img)
        print("Time: %.10f" % (time.time() - start_time))


if __name__ == "__main__":
    main()
