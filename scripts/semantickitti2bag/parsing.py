import numpy as np
import imSLAM
from imSLAM.msg import im_node
from kitti2bag import inv
import pypcd
labelname = "/media/shapelim/UX960NVMe1/kitti_semantic/dataset/sequences/00/labels/000000.label"
labels = np.fromfile(labelname, dtype=np.uint32)
print(labels.shape)
pc = np.fromfile("/media/shapelim/UX960NVMe1/kitti_semantic/dataset/sequences/00/velodyne/000000.bin", dtype=np.float32).reshape(-1, 4)

print(labels.astype(np.float32).shape)

CAM2LIDAR = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, - 7.337429464231e-02],
                      [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                     [0, 0, 0, 1]])

CAM2BASE = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                      [- 6.481465826011e-03, 8.051860151134e-03, - 9.999466081774e-01, 1.65],
                      [9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                      [0, 0, 0, 1]])

LiDAR2BASE = np.array([[0., 0., 0., 0.],
                       [0., 0., 0., 0.],
                       [0., 0., 0., -1.73],
                       [0., 0., 0., 1.]])

a = imSLAM.msg.im_node()

# print(np.linalg.inv(CAM2LIDAR))
print(inv(CAM2LIDAR))
test = np.array([[1],
                 [0],
                 [0],
                 [1]])
print(np.matmul(inv(CAM2LIDAR), test))

# print(np.matmul(CAM2LIDAR, LiDAR2BASE))
# self.T_w_cam0 = []
#             with open(pose_file, 'r') as f:
#                 for line in f.readlines():
#                     T = np.fromstring(line, dtype=float, sep=' ')
#                     T = T.reshape(3, 4)
#                     T = np.vstack((T, [0, 0, 0, 1]))
#                     self.T_w_cam0.append(T)

# scan = (np.fromfile(pc, dtype=np.float32)).reshape(-1, 4)


# print(labels.shape)
# print(pc.shape[0]/4)
# print(labels[:10])
# print("=============")
# # print(np.amax(labels))
# # print(np.amin(labels))
# print(np.unique(labels))
#
#
label = np.fromfile(labelname, dtype=np.uint32)
# label = label.reshape((-1))
# if not isinstance(label, np.ndarray):
#     raise TypeError("Label should be numpy array")
#
# # only fill in attribute if the right size
sem_label = label & 0xFFFF  # semantic label in lower half
inst_label = label >> 16  # instance id in upper half
print(sem_label.shape)
print(inst_label.shape)
#

with open("test.csv", "w") as csvfile:
    pass
# # sanity check
# assert ((sem_label + (inst_label << 16) == label).all())
# print(np.unique(sem_label))
# print(np.unique(inst_label))
#
# print(np.unique(sem_label).shape[0])
# print(np.unique(inst_label).shape[0])
#
# print("=============")
