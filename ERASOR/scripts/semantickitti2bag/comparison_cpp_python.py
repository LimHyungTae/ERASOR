import numpy as np
import pandas as pd
DIR = "/media/shapelim/UX960NVMe1/kitti_semantic/dataset"

for i in range(10):
    cppname = DIR + "/debug_cpp/{}.csv".format(i)
    pythonname = DIR + "/debug_python/{}.csv".format(i)
    cpp_csv = pd.read_csv(cppname)
    python_csv = pd.read_csv(pythonname)
    print(cpp_csv.shape, "vs ", python_csv.shape)
