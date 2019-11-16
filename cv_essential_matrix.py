import cv2
import numpy as np
from matplotlib import pyplot as plt
import se3

for i in range(90, 4000):
    print "image: ", i
    name_im1 = "%06d" % i
    name_im2 = "%06d" % (i + 1)
    im1 = cv2.imread("/media/cs4li/DATADisk3/KITTI/dataset/sequences/00/image_0/%s.png" % name_im1, cv2.COLOR_BGR2GRAY)
    im2 = cv2.imread("/media/cs4li/DATADisk3/KITTI/dataset/sequences/00/image_0/%s.png" % name_im2, cv2.COLOR_BGR2GRAY)

    K = [718.856, 0, 607.1928,
         0, 718.856, 185.2157,
         0, 0, 1]
    K = np.array(K).reshape(3, 3)

    im1_features = np.squeeze(cv2.goodFeaturesToTrack(im1, 150, 0.01, 40))
    # im2_features = np.squeeze(cv2.goodFeaturesToTrack(im2, 150, 0.01, 40))
    im2_features = cv2.calcOpticalFlowPyrLK(im1, im2, im1_features, None, winSize=(21, 21,), maxLevel=3)[0]

    for i in im1_features:
        x, y = i.ravel()
        cv2.circle(im1, (x, y), 3, 255, -1)

    for i in im2_features:
        x, y = i.ravel()
        cv2.circle(im2, (x, y), 3, 255, -1)

    # plt.figure(1)
    # plt.imshow(im1, cmap="gray")

    plt.figure(2)
    plt.imshow(im2, cmap="gray")

    E, mask = cv2.findEssentialMat(im1_features, im2_features, K, cv2.RANSAC, 0.999, 1.0)
    retval, R, t, mask = cv2.recoverPose(E, im1_features, im2_features, K, mask=mask)

    print "R\n", R
    print "t\n", t
    T = np.eye(4, 4)
    T[:3, :3] = R
    T[:3, 3:4] = t
    T = np.linalg.inv(T)

    e = se3.log_SE3(T)
    print "x:%.5f y:%.5f z:%.5f r:%.5f y:%.5f z:%.5f" % (e[0], e[1], e[2], e[3], e[4], e[5])

    # plt.show()

    # cv2.findEssentialMat()
