#!/usr/bin/env python
"""
Take in an image and optionally a lidar scan
Use CNN to do semantic segmantation
Out put a cloud point with semantic color registered
\author Xuan Zhang, David Russell
\date May - July 2018 (X.Z.), Jan 2022 - (D.R.)
"""

from __future__ import division, print_function

import json
import os
import sys
import time

import cv2
import message_filters
import numpy as np
import ros_numpy
import rospy
from color_pcl_generator import ColorPclGenerator, PointType
from cv_bridge import CvBridge, CvBridgeError
from mmseg.apis import inference_segmentor, init_segmentor
from mmseg.core.evaluation import get_palette
from sensor_msgs.msg import Image, PointCloud2
from skimage.transform import resize

# import torch
# from ptsemseg.models import get_model
# from ptsemseg.utils import convert_state_dict


def color_map(N=256, normalized=False):
    """
    Return Color Map in PASCAL VOC format (rgb)
    \param N (int) number of classes
    \param normalized (bool) whether colors are normalized (float 0-1)
    \return (Nx3 numpy array) a color map
    """

    def bitget(byteval, idx):
        return (byteval & (1 << idx)) != 0

    dtype = "float32" if normalized else "uint8"
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7 - j)
            g = g | (bitget(c, 1) << 7 - j)
            b = b | (bitget(c, 2) << 7 - j)
            c = c >> 3
        cmap[i] = np.array([r, g, b])
    cmap = cmap / 255.0 if normalized else cmap
    return cmap


def decode_segmap(temp, n_classes, cmap):
    """
    Given an image of class predictions, produce an bgr8 image with class colors
    \param temp (2d numpy int array) input image with semantic classes (as integer)
    \param n_classes (int) number of classes
    \cmap (Nx3 numpy array) input color map
    \return (numpy array bgr8) the decoded image with class colors
    """
    r = temp.copy()
    g = temp.copy()
    b = temp.copy()
    for l in range(0, n_classes):
        r[temp == l] = cmap[l, 0]
        g[temp == l] = cmap[l, 1]
        b[temp == l] = cmap[l, 2]
    bgr = np.zeros((temp.shape[0], temp.shape[1], 3))
    bgr[:, :, 0] = b
    bgr[:, :, 1] = g
    bgr[:, :, 2] = r
    return bgr.astype(np.uint8)


class SemanticCloud:
    """
    Class for ros node to take in a color image (bgr) and do semantic segmantation on it to produce an image with semantic class colors (chair, desk etc.)
    Then produce point cloud based on depth information
    CNN: PSPNet (https://arxiv.org/abs/1612.01105) (with resnet50) pretrained on ADE20K, fine tuned on SUNRGBD or not
    """

    def __init__(self, gen_pcl=True):
        """
        Constructor
        \param gen_pcl (bool) whether generate point cloud, if set to true the node will subscribe to depth image
        """

        # Get point type
        point_type = rospy.get_param("/semantic_pcl/point_type")
        if point_type == 0:
            self.point_type = PointType.COLOR
            print("Generate color point cloud.")
        elif point_type == 1:
            self.point_type = PointType.SEMANTICS_MAX
            print("Generate semantic point cloud [max fusion].")
        elif point_type == 2:
            self.point_type = PointType.SEMANTICS_BAYESIAN
            print("Generate semantic point cloud [bayesian fusion].")
        else:
            print("Invalid point type.")
            return
        # Get image size
        self.img_width, self.img_height = (
            rospy.get_param("/camera/width"),
            rospy.get_param("/camera/height"),
        )
        # TODO update this
        self.cnn_input_size = (self.img_width, self.img_height)

        extrinsics_str = rospy.get_param("/camera/extrinsics")
        # TODO this is a hack to use the json method
        # self.extrinsics = np.fromstring(extrinsics_str)
        self.extrinsics = np.array(json.loads(extrinsics_str), dtype=float)
        if not np.all(self.extrinsics.shape == (4, 4)):
            raise ValueError("Extrinscs are the wrong shape")

        if not np.allclose((det := np.linalg.det(self.extrinsics[:3, :3])), 1):
            raise ValueError(f"Extrinsics do not contain a valid rotation, det = {det}")

        self.n_classes = rospy.get_param("/semantic_pcl/num_classes")
        self.cmap = color_map(
            N=self.n_classes, normalized=False
        )  # Color map for semantic classes

        # Set up CNN is use semantics
        if self.point_type is not PointType.COLOR:
            # Taken from my version
            # TODO convert this to use ros parameter server
            # config_path = os.path.join(
            #    os.path.dirname(__file__), "..", "cfg", "config.json"
            # )

            # with open(config_path) as infile_h:
            #    self.cfg = json.load(infile_h)

            # self.publish_vis = rospy.get_param("publish_vis")

            model_path = rospy.get_param("/semantic_pcl/model_path")
            config_path = rospy.get_param("/semantic_pcl/config_path")
            device = rospy.get_param("/device")
            print("Setting up CNN model...")
            self.model = init_segmentor(config_path, model_path, device=device)
            # End my version

        # Declare array containers
        if self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.semantic_colors = np.zeros(
                (3, self.img_height, self.img_width, 3), dtype=np.uint8
            )  # Numpy array to store 3 decoded semantic images with highest confidences
            self.confidences = np.zeros(
                (3, self.img_height, self.img_width), dtype=np.float32
            )  # Numpy array to store top 3 class confidences
        # Set up ROS
        print("Setting up ROS...")
        self.bridge = (
            CvBridge()
        )  # CvBridge to transform ROS Image message to OpenCV image
        # Semantic image publisher
        self.sem_img_pub = rospy.Publisher(
            "/semantic_pcl/semantic_image", Image, queue_size=1
        )
        # Set up ros image subscriber
        # Set buff_size to average msg size to avoid accumulating delay
        if gen_pcl:
            # Point cloud frame id
            frame_id = rospy.get_param("/semantic_pcl/frame_id")
            # Camera intrinsic matrix
            fx = rospy.get_param("/camera/fx")
            fy = rospy.get_param("/camera/fy")
            cx = rospy.get_param("/camera/cx")
            cy = rospy.get_param("/camera/cy")
            # TODO get the extrinsics

            intrinsic = np.matrix(
                [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
            )
            self.pcl_pub = rospy.Publisher(
                "/semantic_pcl/semantic_pcl", PointCloud2, queue_size=1
            )
            self.color_sub = message_filters.Subscriber(
                rospy.get_param("/semantic_pcl/color_image_topic"),
                Image,
                queue_size=1,
                buff_size=30 * 480 * 640,
            )
            self.lidar_sub = message_filters.Subscriber(
                rospy.get_param("/semantic_pcl/lidar_topic"),
                PointCloud2,
                queue_size=1,
                # buff_size=40 * 480 * 640, # TODO set the buff size
            )  # increase buffer size to avoid delay (despite queue_size = 1)
            # self.depth_sub = message_filters.Subscriber(
            #    rospy.get_param("/semantic_pcl/depth_image_topic"),
            #    Image,
            #    queue_size=1,
            #    buff_size=40 * 480 * 640,
            # )  # increase buffer size to avoid delay (despite queue_size = 1)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.lidar_sub], queue_size=1, slop=0.3
            )  # Take in one color image and one depth image with a limite time gap between message time stamps
            self.ts.registerCallback(self.color_lidar_callback)
            # TODO Consider if something alterative to this needs to be added
            self.cloud_generator = ColorPclGenerator(
                intrinsic, self.img_width, self.img_height, frame_id, self.point_type
            )
        else:
            self.image_sub = rospy.Subscriber(
                rospy.get_param("/semantic_pcl/color_image_topic"),
                Image,
                self.color_callback,
                queue_size=1,
                buff_size=30 * 480 * 640,
            )
        print("Ready.")

        # Initialize the subscribers last or else the callback will trigger
        # when the model hasn't been created
        # self.sub_rectified = rospy.Subscriber(
        #    rospy.get_param["input_topic"], Image, self.image_callback
        # )
        self.pub_predictions = rospy.Publisher(
            "/seg_class_predictions", Image, queue_size=1
        )
        self.pub_vis = rospy.Publisher("/seg_vis", Image, queue_size=1)

    def color_callback(self, color_img_ros):
        """
        Callback function for color image, de semantic segmantation and show the decoded image. For test purpose
        \param color_img_ros (sensor_msgs.Image) input ros color image message
        """
        print("callback")
        try:
            color_img = self.bridge.imgmsg_to_cv2(
                color_img_ros, "bgr8"
            )  # Convert ros msg to numpy array
        except CvBridgeError as e:
            print(e)
        # Do semantic segmantation
        class_probs = self.predict(color_img)
        confidence, label = class_probs.max(1)
        confidence, label = confidence.squeeze(0).numpy(), label.squeeze(0).numpy()
        label = resize(
            label,
            (self.img_height, self.img_width),
            order=0,
            mode="reflect",
            anti_aliasing=False,
            preserve_range=True,
        )  # order = 0, nearest neighbour
        label = label.astype(np.int)
        # Add semantic class colors
        decoded = decode_segmap(
            label, self.n_classes, self.cmap
        )  # Show input image and decoded image
        confidence = resize(
            confidence,
            (self.img_height, self.img_width),
            mode="reflect",
            anti_aliasing=True,
            preserve_range=True,
        )
        cv2.imshow("Camera image", color_img)
        cv2.imshow("confidence", confidence)
        cv2.imshow("Semantic segmantation", decoded)
        cv2.waitKey(3)

    def color_lidar_callback(self, color_img_ros, lidar_ros):
        """
        Callback function to produce point cloud registered with semantic class color based on input color image and depth image
        \param color_img_ros (sensor_msgs.Image) the input color image (bgr8)
        \param lidar_ros (sensor_msgs.PointCloud2) the lidar in its own frame. TODO 
        """
        # Convert ros Image message to numpy array
        try:
            color_img = ros_numpy.numpify(color_img_ros)
            # color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            # TODO
            lidar = ros_numpy.numpify(lidar_ros)
        except CvBridgeError as e:
            print(e)

        lidar_points = np.stack((lidar["x"], lidar["y"], lidar["z"]), axis=1)

        if self.point_type is PointType.COLOR:
            cloud_ros = self.cloud_generator.generate_cloud_color(
                color_img, lidar_points, color_img_ros.header.stamp,
            )
        else:
            # Do semantic segmantation
            if self.point_type is PointType.SEMANTICS_MAX:
                semantic_color, pred_confidence = self.predict_max(color_img)

                # stamp = rospy.Time.now()
                cloud_ros = self.cloud_generator.generate_cloud_semantic_max(
                    color_img,
                    lidar_points,
                    semantic_color,
                    pred_confidence,
                    color_img_ros.header.stamp,  # Used to be rospy.Time.now(), I'm not sure which is better
                    is_lidar=True,
                    extrinsics=self.extrinsics,
                )

            elif self.point_type is PointType.SEMANTICS_BAYESIAN:
                self.predict_bayesian(color_img)
                # Produce point cloud with rgb colors, semantic colors and confidences
                cloud_ros = self.cloud_generator.generate_cloud_semantic_bayesian(
                    color_img,
                    lidar_points,
                    self.semantic_colors,
                    self.confidences,
                    color_img_ros.header.stamp,
                    is_lidar=True,
                    extrinsics=self.extrinsics,
                )

            # Publish semantic image
            if self.sem_img_pub.get_num_connections() > 0:
                if self.point_type is PointType.SEMANTICS_MAX:
                    semantic_color_msg = ros_numpy.msgify(
                        Image, semantic_color, encoding="bgr8"
                    )
                else:
                    semantic_color_msg = ros_numpy.msgify(
                        Image, self.semantic_colors[0], encoding="bgr8"
                    )
                self.sem_img_pub.publish(semantic_color_msg)

        # Publish point cloud
        self.pcl_pub.publish(cloud_ros)

    def predict_max(self, img):
        """
        Do semantic prediction for max fusion
        \param img (numpy array rgb8)
        """
        class_probs = self.predict(img)
        # Take best prediction and confidence
        # TODO Check this matches the previous behavior
        # class_probs.max(1)
        pred_confidence = np.max(class_probs, axis=2)
        pred_label = np.argmax(class_probs, axis=2)

        # pred_confidence = pred_confidence.squeeze(0).cpu().numpy()
        # pred_label = pred_label.squeeze(0).cpu().numpy()
        pred_label = resize(
            pred_label,
            (self.img_height, self.img_width),
            order=0,
            mode="reflect",
            anti_aliasing=False,
            preserve_range=True,
        )  # order = 0, nearest neighbour
        pred_label = pred_label.astype(np.int)
        # Add semantic color
        semantic_color = decode_segmap(pred_label, self.n_classes, self.cmap)
        pred_confidence = resize(
            pred_confidence,
            (self.img_height, self.img_width),
            mode="reflect",
            anti_aliasing=True,
            preserve_range=True,
        )
        return (semantic_color, pred_confidence)

    def predict_bayesian(self, img):
        """
        Do semantic prediction for bayesian fusion
        \param img (numpy array rgb8)
        """
        class_probs = self.predict(img)
        # Take 3 best predictions and their confidences (probabilities)
        pred_confidences, pred_labels = torch.topk(
            input=class_probs, k=3, dim=1, largest=True, sorted=True
        )
        pred_labels = pred_labels.squeeze(0).cpu().numpy()
        pred_confidences = pred_confidences.squeeze(0).cpu().numpy()
        # Resize predicted labels and confidences to original image size
        for i in range(pred_labels.shape[0]):
            pred_labels_resized = resize(
                pred_labels[i],
                (self.img_height, self.img_width),
                order=0,
                mode="reflect",
                anti_aliasing=False,
                preserve_range=True,
            )  # order = 0, nearest neighbour
            pred_labels_resized = pred_labels_resized.astype(np.int)
            # Add semantic class colors
            self.semantic_colors[i] = decode_segmap(
                pred_labels_resized, self.n_classes, self.cmap
            )
        for i in range(pred_confidences.shape[0]):
            self.confidences[i] = resize(
                pred_confidences[i],
                (self.img_height, self.img_width),
                mode="reflect",
                anti_aliasing=True,
                preserve_range=True,
            )

    def predict(self, img):
        """
        Do semantic segmantation
        \param img: (numpy array bgr8) The input cv image
        """
        img = (
            img.copy()
        )  # Make a copy of image because the method will modify the image
        # orig_size = (img.shape[0], img.shape[1]) # Original image size
        # Prepare image: first resize to CNN input size then extract the mean value of SUNRGBD dataset. No normalization
        img = resize(
            img,
            self.cnn_input_size,
            mode="reflect",
            anti_aliasing=True,
            preserve_range=True,
        )  # Give float64

        img = img.astype(np.float32)
        outputs = inference_segmentor(self.model, img, return_probabilities=True)[0]
        return outputs
        # TODO determine if this pre-processing code is needed
        # img -= self.mean
        ## Convert HWC -> CHW
        # img = img.transpose(2, 0, 1)
        ## Convert to tensor
        # img = torch.tensor(img, dtype=torch.float32)
        # img = img.unsqueeze(0)  # Add batch dimension required by CNN
        # with torch.no_grad():
        #    img = img.to(self.device)
        #    # Do inference
        #    since = time.time()
        #    outputs = self.model(img)  # N,C,W,H
        #    # Apply softmax to obtain normalized probabilities
        #    outputs = torch.nn.functional.softmax(outputs, 1)
        #    return outputs


def main(args):
    rospy.init_node("semantic_cloud", anonymous=True)
    seg_cnn = SemanticCloud(gen_pcl=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
