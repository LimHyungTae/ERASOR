import pypcd
from tqdm import tqdm
import numpy as np

from sklearn.neighbors import NearestNeighbors
from tabulate import tabulate

def intensity2labels(intensity_np):
    label = intensity_np.astype(np.uint32)
    sem_label = label & 0xFFFF  # semantic label in lower half
    inst_label = label >> 16  # instance id in upper half
    return sem_label, inst_label

DYNAMIC_CLASSES = [252, 253, 254, 255, 256, 257, 259]

def fetch_dynamic_objects_ids(intensity_np):
    sem_label, inst_label = intensity2labels(intensity_np)

    num_total = 0
    # Count class
    for class_id in DYNAMIC_CLASSES:
        unique_ids = np.unique(inst_label[sem_label == class_id])
        print(class_id, unique_ids)


def count_static_and_dynamic(intensity_np, verbose=False):
    NUM_CLASS_ON_MAP = {252: 0, 253: 0, 254: 0, 255: 0,
                        256: 0, 257: 0, 258: 0, 259: 0,
                        'dynamic': 0, 'static': 0, 'percentage': 0, 'total': 0}
    sem_label, inst_label = intensity2labels(intensity_np)

    if verbose: print("[Debug]: total: {}".format(sem_label.shape[0]))
    num_total = 0
    # Count class
    for class_id in DYNAMIC_CLASSES:
        num_class = np.count_nonzero(sem_label == class_id)
        NUM_CLASS_ON_MAP[class_id] = num_class
        num_total = num_total + num_class

    nd = num_total
    ns = sem_label.shape[0] - nd
    NUM_CLASS_ON_MAP['dynamic'] = nd
    NUM_CLASS_ON_MAP['static'] = ns
    NUM_CLASS_ON_MAP['total'] = ns + nd
    NUM_CLASS_ON_MAP['percentage'] = float(nd)/float(ns)*100
    display = "# Total - {} => Static : {} | # Dynamic: {} | percentage: {}".format(ns + nd, ns, nd,
                                                                                    float(nd)/float(ns) * 100)
    if verbose: print(display)

    return NUM_CLASS_ON_MAP

def data2xyz_np(pc):
    xs = np.reshape(pc.pc_data['x'], (-1, 1))
    ys = np.reshape(pc.pc_data['y'], (-1, 1))
    zs = np.reshape(pc.pc_data['z'], (-1, 1))
    return np.concatenate((xs, ys, zs), axis=1)

def show_num_static_dynamc(path):
    print("On loading data...")
    data = pypcd.PointCloud.from_path(path)
    print("Complete to load data")
    count_static_and_dynamic(data.pc_data['intensity'])

def calc_metrics(gt, estimate, dists, indices, voxelsize):
    num_pc = gt.pc_data['x'].shape[0]
    assert num_pc == dists.shape[0]
    assert num_pc == indices.shape[0]

    # Metrics
    num_geometric_inliers = 0
    num_inlier_dynamic = 0
    num_inlier_static = 0
    num_remainder_dynamic = 0

    num_preserved = 0
    num_semantically_preserved = 0

    # num_deterministic_inliers = {'inliers': 0, 'num_remainder': 0, 'num_preserved': 0}

    DETERMINISTIC_INLIER_THR = 0.01

    gt_sem_label, gt_inst_label = intensity2labels(gt.pc_data['intensity'])
    # By means of indices, the est values are correspond to gt!
    est_sem_label, est_inst_label = intensity2labels(estimate.pc_data['intensity'][indices])

    # 1. deterministic_inliers - no need to spatial comparison
    is_inliers = dists < DETERMINISTIC_INLIER_THR
    num_preserved += np.count_nonzero(is_inliers)
    gt_inliers_semantic = gt_sem_label[is_inliers]
    est_inliers_semantic = gt_sem_label[is_inliers]
    print(num_preserved)

    for gt_sem, est_sem in tqdm(zip(gt_inliers_semantic, est_inliers_semantic)):
        if gt_sem in DYNAMIC_CLASSES:  # dynamic
            pass
        else:
            if est_sem in DYNAMIC_CLASSES:  # dynamic <-> dynamic
                pass
            else:  # static <-> static
                num_semantically_preserved += 1

    print(num_preserved, num_semantically_preserved)

    # over_the_voxelsize = dists > voxelsize * np.sqrt(3)/2  # Absolutely far from the each point
    # within_the_voxelsize = dists < voxelsize * np.sqrt(3)/2
    #
    # num_geometric_inliers = num_pc - np.count_nonzero(over_the_voxelsize)
    # print(num_geometric_inliers)
    #
    # gt_filtered_xyz = data2xyz_np(gt)[within_the_voxelsize]
    # est_corresponding_xyz = data2xyz_np(estimate)[indices]  # Corresponding
    # dists_filtered = dists[within_the_voxelsize]
    #
    # # Those are filtered
    # gt_sem_label, gt_inst_label = intensity2labels(gt.pc_data['intensity'][within_the_voxelsize])
    # est_sem_label, est_inst_label = intensity2labels(estimate.pc_data['intensity'][indices])
    #
    # for i in range(10):
    #     print(dists_filtered[i])
    #     print(gt_filtered_xyz[i, 0])
    #     print(est_corresponding_xyz[i, 0])
    #
    # print(np.amax(dists_filtered))


def calc_naive_preservation(gt, estimate, dists, indices, voxelsize):
    num_pc = gt.pc_data['x'].shape[0]
    assert num_pc == dists.shape[0]
    assert num_pc == indices.shape[0]

    num_preserved = 0
    num_static_preserved = 0
    num_dynamic_preserved = 0

    DETERMINISTIC_INLIER_THR = voxelsize * np.sqrt(3) / 2
    # DETERMINISTIC_INLIER_THR = voxelsize / 2

    gt_sem_label, gt_inst_label = intensity2labels(gt.pc_data['intensity'])
    # By means of indices, the est values are correspond to gt!
    est_sem_label, est_inst_label = intensity2labels(estimate.pc_data['intensity'][indices])

    # 1. deterministic_inliers - no need to spatial comparison
    is_inliers = dists < DETERMINISTIC_INLIER_THR
    num_preserved += np.count_nonzero(is_inliers)
    gt_inliers = gt_sem_label[is_inliers]
    est_inliers = est_sem_label[is_inliers]

    for gt_sem, est_sem in tqdm(zip(gt_inliers, est_inliers)):
        if (gt_sem not in DYNAMIC_CLASSES) and (est_sem not in DYNAMIC_CLASSES):  # both are static
            num_static_preserved += 1

        elif (gt_sem in DYNAMIC_CLASSES) and (est_sem in DYNAMIC_CLASSES):  # both are dynamic
            num_dynamic_preserved += 1
        else:
            pass

    return num_preserved, num_static_preserved, num_dynamic_preserved


def evaluate(gt, estimate, voxelsize=0.2):
    # 0. print current state
    num_gt = count_static_and_dynamic(gt.pc_data['intensity'])
    num_estimate = count_static_and_dynamic(estimate.pc_data['intensity'])

    # 1. Set data to xyz
    gt_xyz = data2xyz_np(gt)
    estimate_xyz = data2xyz_np(estimate)

    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(estimate_xyz)
    # indices: gt -> estimates
    dists, indices = nbrs.kneighbors(gt_xyz)
    num_preserved, num_static_preserved, num_dynamic_preserved = calc_naive_preservation(gt, estimate, np.reshape(dists, -1), np.reshape(indices, -1), voxelsize)
    # Print metrics

    gt_data = [num_gt['static'], num_gt['dynamic'], num_gt['percentage']]
    est_data = [num_estimate['static'], num_estimate['dynamic'], num_estimate['percentage']]

    precision = float(num_gt['dynamic'] - num_estimate['dynamic']) / float(num_gt['total'] - num_estimate['total'] + 0.00000000001)
    recall = float(num_gt['dynamic'] - num_estimate['dynamic']) / float(num_gt['dynamic'])

    pr = float(num_static_preserved) / float(num_gt['static']) * 100
    rr = float(num_gt['dynamic'] - num_dynamic_preserved) / float(num_gt['dynamic']) * 100
    printed_data = gt_data + est_data \
                   + [num_estimate['percentage']/num_gt['percentage']
                      , pr
                      , rr
                      , np.log(num_estimate['percentage']/100) / np.log(num_gt['percentage']/100)
                      , precision
                      , recall
                      , 2 * ( (pr/100) * (rr/100) ) / ((pr/100) + (rr/100))]
    print(tabulate([printed_data], headers=['# s', '# d', '%', '# s out.', '# d out.', '%',
                                             "% / %", 'Preservation', 'rejection', 'REL (log % / log %)', 'precision', 'recall', 'F1'], tablefmt='orgtbl'))
def load_pcd(path):
    print("On loading data...")
    data = pypcd.PointCloud.from_path(path)
    print("Complete to load data")
    return data

if __name__ == "__main__":
    seq = "00"
    print("target: " + seq)
    src_path = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_v2/07_630_to_820_w_interval2_voxel__0_2.pcd"
    # src_path = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/00_4390_to_4530_w_interval2_voxel_0.200000.pcd"
    # target = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/erasor_best_pcd/00_result.pcd"
    # target = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/erasor_best_pcd/00_result_best.pcd"
    # rm3 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/removert_rm3_00_w_label.pcd"
    # rv1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/removert_rv1_00_w_label.pcd"

    target1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/octomap_output/voxel_size_0.05/07_0.050000_1_w_label.pcd"
    target2 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/octomap_output/voxel_size_0.2/07_0.200000_1_w_label.pcd"
    # in case of seq. 01
    # target1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/octomap_output/voxel_size_0.05/01_0.075000_1_w_label.pcd"
    # target2 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/octomap_output/voxel_size_0.2/01_0.200000_1_w_label.pcd"

    # target3 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/octomap_output/insertpc_00_0.100000_1_w_label.pcd"
    # target4 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/octomap_output/insertpc_00_0.050000_1_w_label.pcd"


    gt_data = load_pcd(src_path)
    t1 = load_pcd(target1)
    t2 = load_pcd(target2)


    evaluate(gt_data, t1, voxelsize=0.2)
    evaluate(gt_data, t2, voxelsize=0.2)

    # w_floor = load_pcd(w_floor_path)
    # wo_floor = load_pcd(wo_floor_path)
    # floor = load_pcd(floor_path)

    ##########################################
    # evaluate(gt_data, wo_floor, voxelsize=0.2)
    # evaluate(gt_data, w_floor, voxelsize=0.2)
    # evaluate(gt_data, floor, voxelsize=0.2)
    ##########################################

    # path = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/08_0_to_249_w_interval3_voxel_0_2.pcd"
    # data = load_pcd(path)
    # fetch_dynamic_objects_ids(data.pc_data['intensity'])
    #
