import pypcd


def data_frame_to_point_cloud(df):
    """ create a PointCloud object from a dataframe.
    """
    pc_data = df.to_records(index=False)
    md = {'version': .7,
          'fields': [],
          'size': [],
          'count': [],
          'width': 0,
          'height': 1,
          'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
          'points': 0,
          'type': [],
          'data': 'binary_compressed'}
    md['fields'] = df.columns.tolist()
    for field in md['fields']:
        type_, size_ =\
            pypcd.numpy_type_to_pcd_type[pc_data.dtype.fields[field][0]]
        md['type'].append(type_)
        md['size'].append(size_)
        # TODO handle multicount
        md['count'].append(1)
    md['width'] = len(pc_data)
    md['points'] = len(pc_data)
    pc = pypcd.PointCloud(md, pc_data)
    return pc


def data_frame_to_message(df, stamp=None, frame_id=None):
    pc_data = df.to_records(index=False)
    return pypcd.numpy_pc2.array_to_pointcloud2(pc_data,
                                                stamp=stamp,
                                                frame_id=frame_id)
