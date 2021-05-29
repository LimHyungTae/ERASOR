import numpy as np
from tqdm import tqdm
import pandas as pd
import os
import matplotlib.pyplot as plt
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
    for i in range(10):
        seq_id = str(i).zfill(2)
        plt.figure(figsize=(25, 8.0))
        # SEQUENCE = "/00"
        SEQUENCE = "/" + seq_id
        labeldir = "/media/shapelim/UX960NVMe1/kitti_semantic/dataset/sequences" + SEQUENCE + "/labels"

        NUM_CLASS_COUNT = {252: [], 253: [], 254: [], 255: [],
                           256: [], 257: [], 258: [], 259: [],
                           'Total': []}
        max_num_total = 0
        labels = sorted(os.listdir(labeldir))

        for i in tqdm(range(len(labels))):
            labelname = labels[i]
            abs_labelname = os.path.join(labeldir, labelname)
            label = np.fromfile(abs_labelname, dtype=np.uint32)
            sem_label = label & 0xFFFF  # semantic label in lower half
            inst_label = label >> 16  # instance id in upper half

            num_total = 0
            # Count class
            for class_id in DYNAMIC_CLASSES:
                num_class = np.count_nonzero(sem_label == class_id)
                NUM_CLASS_COUNT[class_id].append(num_class)
                num_total = num_total + num_class

            if max_num_total < num_total:
                max_num_total = num_total
            NUM_CLASS_COUNT['Total'].append(num_total)

        x_range = range(len(labels))
        for id in DYNAMIC_CLASSES:
            plt.plot(x_range, NUM_CLASS_COUNT[id], label=str(id))

        plt.grid(True)
        plt.legend()
        plt.xlabel("Step")
        plt.ylabel("# of classes")
        if len(labels) > 1600:
            inter = 200
        else:
            inter = 100

        plt.xticks(np.arange(0, (len(labels)/inter+1)*inter, inter), rotation=60)
        plt.ylim(0, 2000)
        fig = plt.gcf()
        fig.savefig("viz/partial_" + SEQUENCE[1:] + ".png")


        plt.plot(x_range, NUM_CLASS_COUNT['Total'], label='Total')

        plt.legend()
        plt.ylim(0, max_num_total)
        fig = plt.gcf()
        fig.savefig("viz/" + SEQUENCE[1:] + ".png")






