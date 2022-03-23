from __future__ import division, print_function

from email.mime import image
from enum import Enum
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import PointCloud2, PointField
import matplotlib.pyplot as plt
import imageio

# import time


class PointType(Enum):
    COLOR = 0
    SEMANTICS_MAX = 1
    SEMANTICS_BAYESIAN = 2


def filter_points_to_image_extent(points, img_size, offset=0.5, return_indices=True):
    """Return points which are within an image extents
    
    The returned points will be such that rounding them will allow valid indexing into the image.
    Therefore, the minimum valid points is -0.5 and the maximum is axis_size - 0.5.

    inputs:
        points: np.array
            (2, n). The projected points in image coordinates
        img_size: tuple(int):
            (width, height) in pixels
        offset: float
            How to account for subpixel indexing. 0.5 return indices that are valid after rounding. 
            0.0 will return indices valid after truncation.
        return_indices: bool
            Whether to return the indices that were used
        
    returns:
        np.array:
            Valid points after filtering. (2, n).
    """
    # Account for differences in how matrices and arrays deal with indexing a single dimension
    points = np.asarray(points)

    img_size_x, img_size_y = img_size
    inside = np.logical_and.reduce(
        (
            points[0] > (-offset),
            points[1] > (-offset),
            points[0] < (img_size_x - offset),
            points[1] < (img_size_y - offset),
        )
    )
    points_inside = points[:, inside]

    if return_indices:
        return points_inside, inside

    return points_inside


def sample_points(img, img_points):
    """Sample the values from an image based on coordinates

    inputs:
        img: np.array
            (h, w, 3) or (h, w). The image to sample from
        img_points: np.array
            (2, n). float or int. Points to sample from. Assumed to be (x, y).T

    returns:
        np.array
        Sampled points concatenated vertically
    """
    if isinstance(img_points, np.matrix):
        # Avoid weird behavior
        img_points = np.asarray(img_points)

    # Force non-int points to be ints
    if issubclass(img_points.dtype.type, np.floating):
        img_points = np.round(img_points).astype(np.uint16)

    sampled_values = img[img_points[1], img_points[0]]
    return sampled_values


class ColorPclGenerator:
    """
    Generate a ros point cloud given a color image and a depth image
    \author Xuan Zhang
    \date May - July 2018
    """

    def __init__(
        self,
        intrinsic,
        width=640,
        height=480,
        frame_id="/camera_rgb_optical_frame",
        point_type=PointType.SEMANTICS_BAYESIAN,
    ):
        """
        width: (int) width of input images
        height: (int) height of input images
        """
        self.point_type = point_type
        self.intrinsic = intrinsic
        self.num_semantic_colors = (
            3  # Number of semantic colors to be sent in the message
        )
        self.img_width = width
        self.img_height = height
        # Allocate arrays
        # x_index = np.array([range(width) * height], dtype="<f4")
        # TODO if this is needed, there is certainly a meshgrid way to do it
        # TODO This is concerning, it appears that it may expect some sort of rectangular grid
        # self.xy_index = np.vstack((x_index, y_index)).T  # x,y
        # self.xyd_vect = np.zeros([width * height, 3], dtype="<f4")  # x,y,depth
        # self.XYZ_vect = np.zeros([width * height, 3], dtype="<f4")  # real world coord

        # I think the last element here is actually a zero to keep it aligned properly
        self.bgr0_vect = None  # np.zeros([width * height, 4], dtype="<u1")  # bgr0

        # TODO figure out what these semantic colors are and if they're needed
        # TODO Figure out why this is
        # self.semantic_color_vect = np.zeros([width * height, 4], dtype="<u1")  # bgr0
        # self.semantic_colors_vect = np.zeros(
        #    [width * height, 4 * self.num_semantic_colors], dtype="<u1"
        # )  # bgr0bgr0bgr0 ...
        # self.confidences_vect = np.zeros(
        #    [width * height, self.num_semantic_colors], dtype="<f4"
        # )  # class confidences
        # Prepare ros cloud msg
        # Cloud data is serialized into a contiguous buffer, set fields to specify offsets in buffer
        self.cloud_ros = PointCloud2()
        self.cloud_ros.header.frame_id = frame_id
        # This represents an un-ordered (non-image point cloud)
        self.cloud_ros.height = 1
        # TODO this number will need to be updated since we no longer know how many points we'll have
        self.cloud_ros.width = width * height
        # TODO make sure all of this specification can stay
        self.cloud_ros.fields.append(
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)
        )
        self.cloud_ros.fields.append(
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1)
        )
        self.cloud_ros.fields.append(
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        )
        # TODO figure out how RGB is being encoded if it's just a float
        self.cloud_ros.fields.append(
            PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1)
        )
        if self.point_type is PointType.SEMANTICS_MAX:
            self.cloud_ros.fields.append(
                PointField(
                    name="semantic_color",
                    offset=20,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
            )
            self.cloud_ros.fields.append(
                PointField(
                    name="confidence", offset=24, datatype=PointField.FLOAT32, count=1
                )
            )
        elif self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.cloud_ros.fields.append(
                PointField(
                    name="semantic_color1",
                    offset=32,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
            )
            self.cloud_ros.fields.append(
                PointField(
                    name="semantic_color2",
                    offset=36,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
            )
            self.cloud_ros.fields.append(
                PointField(
                    name="semantic_color3",
                    offset=40,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
            )
            self.cloud_ros.fields.append(
                PointField(
                    name="confidence1", offset=48, datatype=PointField.FLOAT32, count=1
                )
            )
            self.cloud_ros.fields.append(
                PointField(
                    name="confidence2", offset=52, datatype=PointField.FLOAT32, count=1
                )
            )
            self.cloud_ros.fields.append(
                PointField(
                    name="confidence3", offset=56, datatype=PointField.FLOAT32, count=1
                )
            )

        self.cloud_ros.is_bigendian = False
        if self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.cloud_ros.point_step = 16 * 4  # In bytes
        else:
            self.cloud_ros.point_step = 8 * 4  # In bytes
        # TODO check that this still makes sense. I think so, it's just the number of points times their size
        self.cloud_ros.row_step = (
            self.cloud_ros.point_step * self.cloud_ros.width * self.cloud_ros.height
        )
        self.cloud_ros.is_dense = False

    def set_intrinsics(self, intrinsics):
        """
        Set the intrinsics K matrix
        """
        self.intrinsic = intrinsics

    def generate_cloud_data_common_lidar(self, bgr_img, lidar, extrinsics=None):
        """Project lidar points into the image and texture ones which are within the frame of the image 

        Create a 

        bgr_img:
            (numpy array bgr8). (h, w, 3)
        lidar:
            np.array float32 (n, 3). 3D lidar points in the local frame
        extrinsics:
            np.array (4, 4). The relation between the lidar frame and the camera frame
        [x, y, Z] = [X, Y, Z] * intrinsic.T
        """
        if extrinsics is not None:
            lidar_homog = np.concatenate((lidar, np.ones((lidar.shape[0], 1))), axis=1)
            # Perform the matrix multiplication to get the lidar points into the local frame
            lidar_proj = np.dot(extrinsics, lidar_homog.T)
            lidar_transformed = lidar_proj[:3]
        else:
            # Keep the lidar frame the same as it was initially
            lidar_transformed = lidar.T

        # Note that lidar_transformed is (3, n) to facilitate easier multiplication
        in_front_of_camera = lidar_transformed[2] > 0
        # Take only points which are in front of the camera
        lidar_transformed_filtered = lidar_transformed[:, in_front_of_camera]
        # Project each point into the image
        projections_homog = np.dot(
            np.matrix(self.intrinsic), lidar_transformed_filtered
        )
        projections_inhomog = projections_homog[:2] / projections_homog[2]

        # TODO consider using the shape from the current image
        self.image_points, within_bounds = filter_points_to_image_extent(
            projections_inhomog, (self.img_width, self.img_height)
        )
        projections_inhomog = np.asarray(projections_homog)

        # TODO remember that we actually need to keep track of which points we used
        sampled_colors = sample_points(bgr_img, self.image_points)

        self.cloud_ros.width = self.image_points.shape[1]

        # This is in the local robot frame, not the camera frame
        in_front_XYZ = lidar.T[:, in_front_of_camera]
        # Transpose to get it to be (n, 3)
        self.XYZ_vect = in_front_XYZ[:, within_bounds].T
        num_points = sampled_colors.shape[0]

        self.bgr0_vect = np.concatenate(
            (sampled_colors, np.zeros((num_points, 1), dtype=np.uint8)),
            axis=1,
            dtype=np.uint8,
        )

        if self.point_type is PointType.SEMANTICS_BAYESIAN:
            # Why is this ones and not zeros?
            self.ros_data = np.ones(
                (num_points, 16), dtype="<f4"
            )  # [x,y,z,0,bgr0,0,0,0,color0,color1,color2,0,confidence0,confidence1,confidence2,0]
        else:
            self.ros_data = np.ones(
                (num_points, 8), dtype="<f4"
            )  # [x,y,z,0,bgr0,0,0,0] or [x,y,z,0,bgr0,semantics,confidence,0]

        # Concatenate data
        self.ros_data[:, 0:3] = self.XYZ_vect
        self.ros_data[:, 4:5] = self.bgr0_vect.view("<f4")

    def show_point_cloud(self):
        # 3D Plot
        fig = plt.figure()
        ax3D = fig.add_subplot(111, projection="3d")
        # Scale and convert BGR->RGB
        col = np.flip(self.bgr0_vect[:, :3] / 255.0, axis=1)
        x, y, z = self.XYZ_vect.T
        ax3D.scatter(x, y, z, s=10, c=col, marker="o")
        plt.show()

    def generate_cloud_data_common_img(self, bgr_img, depth_img):
        """
        Do depth registration, suppose that rgb_img and depth_img has the same intrinsic
        \param bgr_img (numpy array bgr8)
        \param depth_img (numpy array float32 2d)
        [x, y, Z] = [X, Y, Z] * intrinsic.T
        """
        bgr_img = bgr_img.view("<u1")
        depth_img = depth_img.view("<f4")
        # Add depth information
        self.xyd_vect[:, 0:2] = self.xy_index * depth_img.reshape(-1, 1)
        self.xyd_vect[:, 2:3] = depth_img.reshape(-1, 1)
        self.XYZ_vect = self.xyd_vect.dot(self.intrinsic.I.T)
        # Convert to ROS point cloud message in a vectorialized manner
        # ros msg data: [x,y,z,0,bgr0,0,0,0,color0,color1,color2,0,confidence0,confidence1,confidenc2,0] (little endian float32)
        # Transform color
        self.bgr0_vect[:, 0:1] = bgr_img[:, :, 0].reshape(-1, 1)
        self.bgr0_vect[:, 1:2] = bgr_img[:, :, 1].reshape(-1, 1)
        self.bgr0_vect[:, 2:3] = bgr_img[:, :, 2].reshape(-1, 1)
        # Concatenate data
        self.ros_data[:, 0:3] = self.XYZ_vect
        self.ros_data[:, 4:5] = self.bgr0_vect.view("<f4")

    def make_ros_cloud(self, stamp):
        # Assign data to ros msg
        # We should send directly in bytes, send in as a list is too slow, numpy tobytes is too slow, takes 0.3s.
        self.cloud_ros.data = self.ros_data.ravel().tobytes()
        self.cloud_ros.header.stamp = stamp
        return self.cloud_ros

    def generate_cloud_color(
        self, bgr_img, three_d_data, stamp, is_lidar=True, extrinsics=None,
    ):
        """
        Generate color point cloud
        \param bgr_img (numpy array bgr8) input color image
        \param depth_img (numpy array float32) input depth image
        is_lidar: bool
            Interpret three_d_data as lidar rather than a depth image 
        """
        if is_lidar:
            self.generate_cloud_data_common_lidar(bgr_img, three_d_data, extrinsics)
        else:
            self.generate_cloud_data_common_img(bgr_img, three_d_data)

        return self.make_ros_cloud(stamp)

    def generate_cloud_semantic_max(
        self,
        bgr_img,
        three_d_data,
        semantic_color,
        confidence,
        stamp,
        is_lidar=True,
        extrinsics=None,
        rotate_img_180: bool = True,
    ):
        """ Produce a max confidence image

        inputs:
            semantic_color:
                (h, w, 3) image representing the colors of the classes
            confidence: np.array
                (w, h) confidence of the max class
            rotate_180:
                Whether to rotate both the semantic and color images 

        """
        if rotate_img_180:
            bgr_img, semantic_color = [
                np.flip(x, (0, 1)) for x in (bgr_img, semantic_color)
            ]

        if is_lidar:
            self.generate_cloud_data_common_lidar(bgr_img, three_d_data, extrinsics)
        else:
            self.generate_cloud_data_common_img(bgr_img, three_d_data)

        # Transform semantic color
        self.semantic_color_vect = sample_points(
            semantic_color, self.image_points
        ).astype("<u1")

        self.semantic_color_vect = np.concatenate(
            (
                self.semantic_color_vect,
                np.zeros((self.semantic_color_vect.shape[0], 1), dtype="<u1"),
            ),
            axis=1,
        )

        confidence = sample_points(confidence, self.image_points)

        # Concatenate data
        self.ros_data[:, 5:6] = self.semantic_color_vect.view("<f4")
        self.ros_data[:, 6] = confidence
        return self.make_ros_cloud(stamp)

    def generate_cloud_semantic_bayesian(
        self,
        bgr_img,
        three_d_data,
        semantic_colors,
        confidences,
        stamp,
        is_lidar=True,
        extrinsics=None,
        rotate_img_180: bool = True,
    ):
        """
        Generate semantic point cloud to be used to do bayesian fusion
        \param bgr_img (numpy array bgr8) input color image
        \param depth_img (numpy array float32) input depth image
        \param semantic_colors (list of bgr8 images) semantic colors of different levels of confidences, ordered by confidences (desc)
        \param confidences (a list of numpy array float32) confidence maps of associated semantic colors, ordered by values (desc)
        \stamp (ros time stamp)
        """
        # TODO check if this is correct
        if rotate_img_180:
            bgr_img = np.flip(bgr_img, (0, 1))
            semantic_colors = np.flip(semantic_colors, (1, 2))

        if is_lidar:
            self.generate_cloud_data_common_lidar(bgr_img, three_d_data, extrinsics)
        else:
            self.generate_cloud_data_common_img(bgr_img, three_d_data)

        # Transform semantic colors
        num_points_in_image = self.image_points.shape[1]
        self.semantic_colors_vect = np.zeros(
            (num_points_in_image, 4 * self.num_semantic_colors), dtype="<u1"
        )
        self.confidences_vect = np.zeros(
            [num_points_in_image, self.num_semantic_colors], dtype="<f4"
        )  # class confidences

        for i in range(self.num_semantic_colors):
            first_channel = semantic_colors[i][:, :, 0]
            second_channel = semantic_colors[i][:, :, 1]
            third_channel = semantic_colors[i][:, :, 2]

            sampled_first_channel = sample_points(first_channel, self.image_points)
            sampled_second_channel = sample_points(second_channel, self.image_points)
            sampled_third_channel = sample_points(third_channel, self.image_points)

            self.semantic_colors_vect[:, 4 * i] = sampled_first_channel
            self.semantic_colors_vect[:, 4 * i + 1] = sampled_second_channel
            self.semantic_colors_vect[:, 4 * i + 2] = sampled_third_channel

        # Transform class confidence
        for i in range(self.num_semantic_colors):
            sampled_confidence = sample_points(confidences[i], self.image_points)
            self.confidences_vect[:, i] = sampled_confidence

        # Concatenate data
        self.ros_data[
            :, 8 : 8 + self.num_semantic_colors
        ] = self.semantic_colors_vect.view("<f4")
        self.ros_data[:, 12 : 12 + self.num_semantic_colors] = self.confidences_vect
        return self.make_ros_cloud(stamp)


# Test
if __name__ == "__main__":
    import time

    from matplotlib import pyplot as plt
    from skimage import io

    # Init ros
    rospy.init_node("pcl_test", anonymous=True)
    pcl_pub = rospy.Publisher("/semantic_pcl/semantic_pcl", PointCloud2, queue_size=1)
    # Read test images
    color_img = io.imread(Path(__file__, "../../../pcl_test/color_image.png"))
    depth_img = io.imread(Path(__file__, "../../../pcl_test/depth_image.tiff"))

    # Show test input images
    # plt.ion()
    # plt.show()
    # plt.subplot(1, 2, 1), plt.imshow(color_img[:, :, ::-1]), plt.title("color")
    # plt.subplot(1, 2, 2), plt.imshow(depth_img), plt.title("depth")
    # plt.draw()

    # Camera intrinsic matrix
    fx = 544.771755
    fy = 546.966312
    cx = 322.376103
    cy = 245.357925
    intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    print("intrinsic matrix", intrinsic)

    cloud_gen = ColorPclGenerator(
        intrinsic,
        color_img.shape[1],
        color_img.shape[0],
        point_type=PointType.SEMANTICS_MAX,
    )
    # Generate point cloud and pulish ros message
    while not rospy.is_shutdown():
        lidar_points = (np.random.rand(20000, 3) - 0.5) * 10
        since = time.time()
        stamp = rospy.Time.now()
        # The semantic color is just a color image. Flip the input so it's visually different.
        semantic_color = np.flip(color_img, axis=2)
        confidences = np.ones_like(color_img[..., 0], dtype=float)

        cloud_ros = cloud_gen.generate_cloud_semantic_max(
            color_img,
            lidar_points,
            semantic_color=semantic_color,
            confidence=confidences,
            stamp=stamp,
            is_lidar=True,
        )
        pcl_pub.publish(cloud_ros)
        print("Generate and publish pcl took", time.time() - since)
    rospy.spin()
