
import copy
import numpy as np
from sensor_msgs.msg import PointField

# these are from numpy_pc2
_pftype_to_nptype = dict([(PointField.INT8, np.dtype('int8')),
                          (PointField.UINT8, np.dtype('uint8')),
                          (PointField.INT16, np.dtype('int16')),
                          (PointField.UINT16, np.dtype('uint16')),
                          (PointField.INT32, np.dtype('int32')),
                          (PointField.UINT32, np.dtype('uint32')),
                          (PointField.FLOAT32, np.dtype('float32')),
                          (PointField.FLOAT64, np.dtype('float64'))])

_pftype_to_pcd_letter = dict([(PointField.INT8, 'I'),
                              (PointField.UINT8, 'U'),
                              (PointField.INT16, 'I'),
                              (PointField.UINT16, 'U'),
                              (PointField.INT32, 'I'),
                              (PointField.UINT32, 'U'),
                              (PointField.FLOAT32, 'F'),
                              (PointField.FLOAT64, 'F')])

_pftype_to_size = dict([(PointField.INT8, 1),
                        (PointField.UINT8, 1),
                        (PointField.INT16, 2),
                        (PointField.UINT16, 2),
                        (PointField.INT32, 4),
                        (PointField.UINT32, 4),
                        (PointField.FLOAT32, 4),
                        (PointField.FLOAT64, 8)])


_nea_field_dicts = [
    {'name': 'x',
     'offset': 0,
     'datatype': PointField.FLOAT64,
     'count': 1
     },
    {'name': 'y',
     'offset': 8,
     'datatype': PointField.FLOAT64,
     'count': 1
     },
    {'name': 'z',
     'offset': 16,
     'datatype': PointField.FLOAT64,
     'count': 1
     },
    {'name': 'x_origin',
     'offset': 24,
     'datatype': PointField.FLOAT64,
     'count': 1
     },
    {'name': 'y_origin',
     'offset': 32,
     'datatype': PointField.FLOAT64,
     'count': 1
     },
    {'name': 'z_origin',
     'offset': 40,
     'datatype': PointField.FLOAT64,
     'count': 1
     },
    {'name': 'range_variance',
     'offset': 48,
     'datatype': PointField.FLOAT32,
     'count': 1
     },
    {'name': 'x_variance',
     'offset': 52,
     'datatype': PointField.FLOAT32,
     'count': 1
     },
    {'name': 'y_variance',
     'offset': 56,
     'datatype': PointField.FLOAT32,
     'count': 1
     },
    {'name': 'z_variance',
     'offset': 60,
     'datatype': PointField.FLOAT32,
     'count': 1
     },
    {'name': 'reflectance',
     'offset': 64,
     'datatype': PointField.FLOAT32,
     'count': 1
     },
    {'name': 'time_sec',
     'offset': 68,
     'datatype': PointField.UINT32,
     'count': 1
     },
    {'name': 'time_nsec',
     'offset': 72,
     'datatype': PointField.UINT32,
     'count': 1
     },
    {'name': 'return_type',
     'offset': 76,
     'datatype': PointField.UINT8,
     'count': 1
     }]

_nea_float_fields_dicts = [
                          {'name': 'x',
                           'offset': 0,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'y',
                           'offset': 4,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'z',
                           'offset': 8,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'x_origin',
                           'offset': 12,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'y_origin',
                           'offset': 16,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'z_origin',
                           'offset': 20,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'range_variance',
                           'offset': 24,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'x_variance',
                           'offset': 28,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'y_variance',
                           'offset': 32,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'z_variance',
                           'offset': 36,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'reflectance',
                           'offset': 40,
                           'datatype': PointField.FLOAT32,
                           'count': 1
                           },
                          {'name': 'time_sec',
                           'offset': 44,
                           'datatype': PointField.UINT32,
                           'count': 1
                           },
                          {'name': 'time_nsec',
                           'offset': 48,
                           'datatype': PointField.UINT32,
                           'count': 1
                           },
                          {'name': 'return_type',
                           'offset': 52,
                           'datatype': PointField.UINT8,
                           'count': 1
                           }]


_label_field_dict = {'name': 'label',
                     'offset': 0,
                     'datatype': PointField.UINT8,
                     'count': 1}


# TODO if I use '_' for padding, like pcl, there are weird
# interactions when using binary_compressed. PCL assumes
# padding is not included in compressed files (which makes sense).
_padding_field_dict = {'name': '_PAD',
                       'offset': 0,
                       'datatype': PointField.UINT8,
                       'count': 0}


def datatype_to_size(datatype):
    """ ROS pointfield datatype to size in bytes
    """
    if datatype in (PointField.INT8, PointField.UINT8):
        return 1
    elif datatype in (PointField.INT16, PointField.UINT16):
        return 2
    elif datatype in (PointField.INT32, PointField.UINT32, PointField.FLOAT32):
        return 4
    elif datatype in (PointField.FLOAT64,):
        return 8


def make_nea_fields_dicts(with_label=True, with_padding=True):
    field_dicts = copy.deepcopy(_nea_field_dicts)
    if with_label:
        label_field_dict_copy = copy.deepcopy(_label_field_dict)
        # TODO this assumes last field is return_type
        label_field_dict_copy['offset'] = field_dicts[-1]['offset']+1
        field_dicts.append(label_field_dict_copy)
    if with_padding:
        padding_field_dict_copy = copy.deepcopy(_padding_field_dict)
        # TODO this assumes last field is return_type or label
        padding_field_dict_copy['offset'] = field_dicts[-1]['offset']+1
        if with_label:
            padding_field_dict_copy['count'] = 2
        else:
            padding_field_dict_copy['count'] = 3
        field_dicts.append(padding_field_dict_copy)
    return field_dicts


def make_nea_float_fields_dicts(with_label=True, with_padding=True):
    field_dicts = copy.deepcopy(_nea_float_fields_dicts)
    if with_label:
        label_field_dict_copy = copy.deepcopy(_label_field_dict)
        # TODO this assumes last field is return_type
        label_field_dict_copy['offset'] = field_dicts[-1]['offset']+1
        field_dicts.append(label_field_dict_copy)
    if with_padding:
        padding_field_dict_copy = copy.deepcopy(_padding_field_dict)
        # TODO this assumes last field is return_type or label
        padding_field_dict_copy['offset'] = field_dicts[-1]['offset']+1
        if with_label:
            padding_field_dict_copy['count'] = 9
        else:
            padding_field_dict_copy['count'] = 10
        field_dicts.append(padding_field_dict_copy)
    return field_dicts


def field_dict_list_to_dtypes(field_dicts):
    dtypes = []
    for f in field_dicts:
        count = f['count']
        if count > 1:
            for c in xrange(count):
                name = '%s_%04d' % (f['name'], c)
                nptype = _pftype_to_nptype[f['datatype']]
                dtypes.append((name, nptype))
        else:
            name = f['name']
            nptype = _pftype_to_nptype[f['datatype']]
            dtypes.append((name, nptype))
    return dtypes


def make_nea_dtypes(with_label=True, with_padding=True):
    fl = make_nea_fields_dicts(with_label=with_label,
                               with_padding=with_padding)
    return field_dict_list_to_dtypes(fl)


def make_nea_float_dtypes(with_label=True, with_padding=True):
    fl = make_nea_float_fields_dicts(with_label=with_label,
                                     with_padding=with_padding)
    return field_dict_list_to_dtypes(fl)


def field_dict_list_to_pcd_metadata(field_dict_list):
    pcd_md = {'version': .7,
              'fields': [f['name'] for f in field_dict_list],
              'size': [_pftype_to_size[f['datatype']] for f in
                       field_dict_list],
              'type': [_pftype_to_pcd_letter[f['datatype']] for f in
                       field_dict_list],
              'count': [f['count'] for f in field_dict_list],
              'width': 0,
              'height': 1,
              'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
              'points': 0,
              'data': 'ASCII'}
    return pcd_md

# nea_fields = [ PointField(**d) for d in nea_fields_dicts]
