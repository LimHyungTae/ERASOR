import numpy as np
from tqdm import tqdm
import pandas as pd
import os
import scipy.io
import matplotlib as mpl
# 252: "moving-car"
# 253: "moving-bicyclist"
# 254: "moving-person"
# 255: "moving-motorcyclist"
# 256: "moving-on-rails"
# 257: "moving-bus"
# 258: "moving-truck"
# 259: "moving-other-vehicle"
DYNAMIC_CLASSES = [252, 253, 254, 255, 256, 257, 259]


if __name__ == "__main__":
    for i in range(0, 11):
        seq_id = str(i).zfill(2)
        # SEQUENCE = "/00"
        SEQUENCE = "/" + seq_id
        labeldir = "/media/shapelim/UX960NVMe1/kitti_semantic/dataset/sequences" + SEQUENCE + "/labels"
        velodir = "/media/shapelim/UX960NVMe1/kitti_semantic/dataset/sequences" + SEQUENCE + "/velodyne"

        NUM_CLASS_COUNT = {252: [], 253: [], 254: [], 255: [],
                           256: [], 257: [], 258: [], 259: [],
                           'Total': []}

        INTENSITIES = np.array([])

        labels = sorted(os.listdir(labeldir))
        pcs = sorted(os.listdir(velodir))

        for i in tqdm(range(len(labels))):
            labelname = labels[i]
            pcname = pcs[i]
            assert (labelname[:-len(".label")] == pcname[:-len(".bin")])

            abs_labelname = os.path.join(labeldir, labelname)
            label = np.fromfile(abs_labelname, dtype=np.uint32)
            sem_label = label & 0xFFFF  # semantic label in lower half
            inst_label = label >> 16  # instance id in upper half

            abs_pcname = os.path.join(velodir, pcname)
            scan = np.fromfile(abs_pcname, dtype=np.float32)
            intensity = scan.reshape((-1, 4))[:, 3]

            for class_id in DYNAMIC_CLASSES:
                is_dynamic = sem_label == class_id
                target_intensity = intensity[is_dynamic]
                if target_intensity.shape[0] != 0:
                    INTENSITIES = np.concatenate((INTENSITIES, target_intensity))
                    print(INTENSITIES.shape)

        mat_filename = seq_id + "_intensity.mat"
        scipy.io.savemat(mat_filename, {'intensity': INTENSITIES})


