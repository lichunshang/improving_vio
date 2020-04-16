from __future__ import absolute_import, division, print_function
from copy import deepcopy
# from model_pwcnet import ModelPWCNet, _DEFAULT_PWCNET_TEST_OPTIONS
from tfoptflow.tfoptflow.model_pwcnet import ModelPWCNet, _DEFAULT_PWCNET_TEST_OPTIONS
import skimage.io
import cv2
import matplotlib.pyplot as plt
import numpy as np
import tfoptflow.tfoptflow.visualize as visualize
import os
from matplotlib.blocking_input import BlockingInput
import natsort


class BlockingKeyInput(BlockingInput):
    """
    Callable for retrieving mouse clicks and key presses in a blocking way.
    """

    def __init__(self, fig):
        # BlockingInput.__init__(self, fig=fig, eventslist=(
        #     'button_press_event', 'key_press_event'))
        BlockingInput.__init__(self, fig=fig, eventslist=('key_press_event',))
        self.event_key = None

    def post_event(self):
        """Determine if it is a key event."""
        if self.events:
            self.keyormouse = self.events[-1].name == 'key_press_event'
            self.event_key = self.events[-1].key

    def __call__(self, timeout=30):
        """
        Blocking call to retrieve a single mouse click or key press.

        Returns ``True`` if key press, ``False`` if mouse click, or ``None`` if
        timed out.
        """
        self.keyormouse = None
        BlockingInput.__call__(self, n=1, timeout=timeout)

        return self.event_key


class Undistorter(object):
    def __init__(self, directory):
        if "EUROC" in directory:
            self.K = np.array([
                [4.616e+02, 0, 3.630e+02],
                [0, 4.603e+02, 2.481e+02],
                [0, 0, 1]
            ])
            self.D = np.array([-2.917e-01, 8.228e-02, 5.333e-05, -1.578e-04])
            self.thres = 3.0
            self.distortion_type = "pinhole"
        elif "UZH" in directory and "45" in directory:
            self.K = np.array([
                [275.46015578667294, 0, 274.9948095922592],
                [0, 315.958384100568, 242.7123497822731],
                [0, 0, 1]
            ])
            self.thres = 3.0
            self.D = np.array([-6.545154718304953e-06, -0.010379525898159981, 0.014935312423953146, -0.005639061406567785])
            self.distortion_type = "equidistant"
        else:
            self.K = np.array([
                [190.97847715128717, 0, 254.93170605935475],
                [0, 190.9733070521226, 256.8974428996504],
                [0, 0, 1]
            ])
            self.thres = 3.0
            self.D = np.array([0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182])
            self.distortion_type = "equidistant"
            # raise NotImplementedError("Invalid dir %s" % directory)



    def reject_wit_F(self, curr_pts, next_pts):
        if self.distortion_type == "pinhole":
            cur_pts_undistorted = cv2.undistortPoints(np.expand_dims(curr_pts, 0), self.K, self.D, P=self.K.copy())
            nxt_pts_klt_undistorted = cv2.undistortPoints(np.expand_dims(next_pts, 0), self.K, self.D, P=self.K.copy())
            print("Pinhole!")
        elif self.distortion_type == "equidistant":
            cur_pts_undistorted = cv2.fisheye.undistortPoints(np.expand_dims(curr_pts, 0), self.K, self.D, P=self.K.copy())
            nxt_pts_klt_undistorted = cv2.fisheye.undistortPoints(np.expand_dims(next_pts, 0), self.K, self.D, P=self.K.copy())
            print("fisheye!")
        else:
            raise NotImplementedError("Invalid distortion type %s" % self.distortion_type)

        retval, mask = cv2.findFundamentalMat(cur_pts_undistorted, nxt_pts_klt_undistorted, cv2.FM_RANSAC,
                                              param1=self.thres, param2=0.99)

        cur_pts_non_outlier = curr_pts[mask.squeeze() == 1].squeeze()
        next_pts_klt_non_outlier = next_pts[mask.squeeze() == 1].squeeze()

        return cur_pts_undistorted, nxt_pts_klt_undistorted, cur_pts_non_outlier, next_pts_klt_non_outlier, mask


class Compare(object):
    def __init__(self):
        # Here, we're using a GPU (use '/device:CPU:0' to run inference on the CPU)
        gpu_devices = ['/device:GPU:1']
        controller = '/device:GPU:1'

        # More options...
        # Set the number of samples to visually inspect after inference
        # Set the path to the trained model (make sure you've downloaded it first from http://bit.ly/tfoptflow)
        # Set the batch size
        # ckpt_path = './tfoptflow/tfoptflow/models/pwcnet-lg-6-2-multisteps-chairsthingsmix/pwcnet.ckpt-595000'
        ckpt_path = './tfoptflow/tfoptflow/models/pwcnet-sm-6-2-multisteps-chairsthingsmix/pwcnet.ckpt-592000'
        # ckpt_path = '/home/cs4li/Dev/dump/kitti_ft/pwcnet.ckpt-423000'
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
        nn_opts['use_dense_cx'] = False
        nn_opts['use_res_cx'] = False
        nn_opts['pyr_lvls'] = 6
        nn_opts['flow_pred_lvl'] = 2

        # The size of the images in this dataset are not multiples of 64, while the model generates flows padded to multiples
        # of 64. Hence, we need to crop the predicted flows to their original size
        nn_opts['adapt_info'] = (1, 436, 1024, 2)

        # Instantiate the model in inference mode and display the model configuration
        self.nn = ModelPWCNet(mode='test', options=nn_opts, dataset=None)
        self.nn.print_config()

    def compute_nn_flow(self, im1, im2, cur_pts):
        im1 = np.stack([im1, im1, im1], axis=-1)
        im2 = np.stack([im2, im2, im2], axis=-1)
        ret = self.nn.predict_from_img_pairs(((im1, im2,),), batch_size=1, verbose=False)
        flow_sampled = ret[0][cur_pts.astype(np.uint)[:, 1], cur_pts.astype(np.uint)[:, 0], :]

        return cur_pts + flow_sampled, ret[0]

    def compare_optical_flow(self, plt_name, im1, im2, fig, ax, undistorter):
        im1_cv = skimage.img_as_ubyte(im1)
        im2_cv = skimage.img_as_ubyte(im2)

        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        im1_cv = clahe.apply(im1_cv)
        im2_cv = clahe.apply(im2_cv)

        cur_pts = cv2.goodFeaturesToTrack(im1_cv, 150, 0.01, 25)
        nxt_pts_klt, status, err = cv2.calcOpticalFlowPyrLK(im1_cv, im2_cv, cur_pts, None, winSize=(21, 21,),
                                                            maxLevel=3)

        cur_pts = cur_pts.squeeze()
        nxt_pts_nn, flow = self.compute_nn_flow(im1_cv, im2_cv, cur_pts)
        nxt_pts_lkr, status_lkr, err_lkr = cv2.calcOpticalFlowPyrLK(im1_cv, im2_cv, cur_pts, nxt_pts_nn.copy(),
                                                                    winSize=(21, 21,),
                                                                    maxLevel=3, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)

        nxt_pts_klt = nxt_pts_klt.squeeze()
        nxt_pts_klt = nxt_pts_klt[status.squeeze() == 1]
        cur_pts_filtered = cur_pts[status.squeeze() == 1]

        # nxt_pts_klt = nxt_pts_nn
        # cur_pts_filtered = cur_pts

        _, _, cur_pts_klt_non_outlier, nxt_pts_klt_non_outlier, _ = undistorter.reject_wit_F(cur_pts_filtered,
                                                                                             nxt_pts_klt)

        ax[0, 0].clear()
        ax[0, 0].imshow(im1_cv, cmap='gray')
        ax[0, 0].scatter(cur_pts[:, 0], cur_pts[:, 1], s=3, c="b")

        ax[0, 1].clear()
        ax[0, 1].imshow(im2_cv, cmap='gray')
        ax[0, 1].scatter(cur_pts[:, 0], cur_pts[:, 1], s=3, c="b")
        ax[0, 1].scatter(nxt_pts_klt[:, 0], nxt_pts_klt[:, 1], s=3, c="r")
        ax[0, 1].scatter(nxt_pts_nn[:, 0], nxt_pts_nn[:, 1], s=3, c="g")
        # ax[0, 1].scatter(nxt_pts_lkr[:, 0], nxt_pts_lkr[:, 1], s=3, c="y")

        ax[1, 0].clear()
        ax[1, 0].imshow((im1_cv.astype(np.float32) * 0.5 + im2_cv.astype(np.float32) * 0.5).astype(np.uint8),
                        cmap='gray')
        ax[1, 0].scatter(cur_pts[:, 0], cur_pts[:, 1], s=3, c="b")
        # for i in range(0, len(nxt_pts_klt)):
        #     ax[1, 0].annotate("", xy=nxt_pts_klt[i], xytext=cur_pts_filtered[i],
        #                       arrowprops=dict(arrowstyle="->", color="y", linewidth=1))

        # for i in range(0, len(nxt_pts_klt_non_outlier)):
        #     ax[1, 0].annotate("", xy=nxt_pts_klt_non_outlier[i], xytext=cur_pts_klt_non_outlier[i],
        #                       arrowprops=dict(arrowstyle="->", color="r", linewidth=1))

        for i in range(0, len(nxt_pts_klt)):
            ax[1, 0].annotate("", xy=nxt_pts_klt[i], xytext=cur_pts_filtered[i],
                              arrowprops=dict(arrowstyle="->", color="r", linewidth=1.5))

        for i in range(0, len(nxt_pts_nn)):
            ax[1, 0].annotate("", xy=nxt_pts_nn[i], xytext=cur_pts[i],
                              arrowprops=dict(arrowstyle="->", color="g", linewidth=1.5))

        # for i in range(0, len(nxt_pts_lkr)):
        #     ax[1, 0].annotate("", xy=nxt_pts_lkr[i], xytext=cur_pts[i],
        #                       arrowprops=dict(arrowstyle="->", color="y", linewidth=1))

        ax[1, 1].clear()
        ax[1, 1].imshow(visualize.flow_to_img(flow))
        # ax[1, 1].imshow(np.linalg.norm(flow, axis=2) * 10)
        subsample_every_N = 20
        quiver_X = np.arange(0, im1_cv.shape[0], subsample_every_N)
        quiver_Y = np.arange(0, im1_cv.shape[1], subsample_every_N)
        # mesh = np.meshgrid(quiver_X, quiver_Y)
        flow_subsampled = flow[::subsample_every_N, ::subsample_every_N]
        quiver_U = flow_subsampled[:, :, 0]
        quiver_V = -flow_subsampled[:, :, 1]
        ax[1, 1].quiver(quiver_Y, quiver_X, quiver_U, quiver_V)

        ax[0, 0].set_xlim(0, flow.shape[1])
        ax[0, 0].set_ylim(0, flow.shape[0])
        ax[0, 0].invert_yaxis()

        plt.gcf().suptitle(plt_name)

        plt.draw()
        plt.pause(0.001)
        blocking = BlockingKeyInput(fig=plt.gcf())
        key = blocking(timeout=-1)

        return key


c = Compare()

fig, ax = plt.subplots(2, 2, sharex=True, sharey=True)

# directory = "/home/cs4li/Dev/EUROC/V2_03_difficult/mav0/cam0/data"
# directory = "/home/cs4li/Dev/EUROC/V2_02_medium/mav0/cam0/data"
# directory = "/home/cs4li/Dev/EUROC/V1_03_difficult/mav0/cam0/data"
# directory = "/home/cs4nli/Dev/TUMVIO/dataset-corridor1_512_16/mav0/cam0/data"
# directory = "/home/cs4li/Dev/TUMVIO/dataset-corridor1_512_16/mav0/cam0/data"
directory = "/home/cs4li/Dev/TUMVIO/dataset-outdoors2_512_16/mav0/cam0/data"
# directory = "/home/cs4li/Dev/TUMVIO/dataset-outdoors6_512_16/mav0/cam0/data"
# directory = "/home/cs4li/Dev/UZH_FPV/indoor_45_13_snapdragon_with_gt/img"
# directory = "/home/cs4li/Dev/UZH_FPV/outdoor_45_1_snapdragon_with_gt/img"
# directory = "/home/cs4li/Dev/UZH_FPV/indoor_45_9_snapdragon_with_gt/img"
undistorter = Undistorter(directory)
every_N_frames = 1
imgs = natsort.natsorted(os.listdir(directory))

prev_img = None
prev_img_name = ""
prev_img_idx = 0
i = 15672
while i < len(imgs):
    if prev_img is not None:
        im = skimage.io.imread(os.path.join(directory, imgs[i]))
        print("reading " + os.path.join(directory, imgs[i]))
        plt_name = "%s: %s -> %s, %s -> %s" % (directory, prev_img_name, imgs[i], prev_img_idx, i)
        key = c.compare_optical_flow(plt_name, prev_img, im, fig, ax, undistorter)
        prev_img = im
        prev_img_name = imgs[i]
        prev_img_idx = i
        if key == "v":
            i += 2
        elif key == "b":
            i += 3
        elif key == "m":
            i += 10
            print("Skip 10")
        elif key == ",":
            i += 100
            print("Skip 100")
        elif key == ".":
            i += 1000
            print("Skip 1000")
        elif key == "/":
            i += 10000
            print("Skip 10000")
        else:
            i += every_N_frames

    if prev_img is None:
        prev_img = skimage.io.imread(os.path.join(directory, imgs[i]))
        prev_img_name = imgs[i]
        prev_img_idx = i
        i += every_N_frames
