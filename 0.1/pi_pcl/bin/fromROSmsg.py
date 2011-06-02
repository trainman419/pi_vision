import roslib; roslib.load_manifest('sensor_msgs')
import math
import struct
import rosbag
from sensor_msgs.msg import PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False):
    assert(cloud)
    fmt = _get_struct_fmt(cloud, field_names)
    width, height, point_step, row_step, data, unpack_from, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, struct.unpack_from, math.isnan
    if skip_nans:
        for v in xrange(height):
            offset = row_step * v
            for u in xrange(width):
                p = unpack_from(fmt, data, offset)
                has_nan = False
                for v in p:
                    if isnan(v):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
                offset += point_step
    else:
        for v in xrange(height):
            offset = row_step * v
            for u in xrange(width):
                yield unpack_from(fmt, data, offset)
                offset += point_step

def _get_struct_fmt(cloud, field_names=None):
    fmt = '>' if cloud.is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt
