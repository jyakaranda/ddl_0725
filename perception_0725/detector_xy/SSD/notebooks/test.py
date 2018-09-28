import os
import math
import random
import time
import numpy as np
import tensorflow as tf
import cv2

slim = tf.contrib.slim


import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import sys
sys.path.append('../')

from nets import ssd_vgg_300, ssd_common, np_methods
from preprocessing import ssd_vgg_preprocessing
from notebooks import visualization



gpu_options = tf.GPUOptions(allow_growth=True)
config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
isess = tf.InteractiveSession(config=config)


net_shape = (300, 300)
data_format = 'NHWC'
img_input = tf.placeholder(tf.uint8, shape=(None, None, 3))

image_pre, labels_pre, bboxes_pre, bbox_img = ssd_vgg_preprocessing.preprocess_for_eval(
    img_input, None, None, net_shape, data_format, resize=ssd_vgg_preprocessing.Resize.WARP_RESIZE)
image_4d = tf.expand_dims(image_pre, 0)


reuse = True if 'ssd_net' in locals() else None
ssd_net = ssd_vgg_300.SSDNet()
with slim.arg_scope(ssd_net.arg_scope(data_format=data_format)):
    predictions, localisations, _, _ = ssd_net.net(image_4d, is_training=False, reuse=reuse)


ckpt_filename = '../checkpoints/ssd_300_vgg.ckpt'

print(isess.run(tf.global_variables_initializer()))
saver = tf.train.Saver()
saver.restore(isess, ckpt_filename)


ssd_anchors = ssd_net.anchors(net_shape)



def process_image(img, select_threshold=0.5, nms_threshold=.45, net_shape=(300, 300)):

    rimg, rpredictions, rlocalisations, rbbox_img = isess.run([image_4d, predictions, localisations, bbox_img],
                                                              feed_dict={img_input: img})


    rclasses, rscores, rbboxes = np_methods.ssd_bboxes_select(
        rpredictions, rlocalisations, ssd_anchors,
        select_threshold=select_threshold, img_shape=net_shape, num_classes=21, decode=True)

    rbboxes = np_methods.bboxes_clip(rbbox_img, rbboxes)
    rclasses, rscores, rbboxes = np_methods.bboxes_sort(rclasses, rscores, rbboxes, top_k=400)
    rclasses, rscores, rbboxes = np_methods.bboxes_nms(rclasses, rscores, rbboxes, nms_threshold=nms_threshold)

    rbboxes = np_methods.bboxes_resize(rbbox_img, rbboxes)
    return rclasses, rscores, rbboxes



#import time
#path = '../demo/'
#image_names = sorted(os.listdir(path))
#print("len(image_names)",len(image_names))



cap = cv2.VideoCapture(0)
fps = cap.get(cv2.CAP_PROP_FPS)
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
fourcc = cap.get(cv2.CAP_PROP_FOURCC)

print('fps=%d,size=%r,fourcc=%r'%(fps,size,fourcc))
delay=60/int(fps)

while(cap.isOpened()):
      ret,frame = cap.read()
      if ret==True:
          image = frame
          #image_np = image
          #cv2.imshow('frame', image_np)
          # 376 1344
          # print("frame.shape[0]   frame.shape[1]",image_np.shape[0],image_np.shape[1])

          image1 = image[:,:672] # 左
          image2 = image[:,672:] # 右

          rclasses1, rscores1, rbboxes1 = process_image(image1)
          rclasses2, rscores2, rbboxes2 = process_image(image2)


          visualization.bboxes_draw_on_img(image1, rclasses1, rscores1, rbboxes1)
          visualization.bboxes_draw_on_img(image2, rclasses2, rscores2, rbboxes2)

          cv2.imshow('image1', image1)
          cv2.imshow('image2', image2)



          cv2.waitKey(np.uint(delay))

      else:
          break
cap.release()
cv2.destroyAllWindows()










