"""Decode strided PointCloud2 layouts and preserve known XT32 beam geometry."""
from array import array
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
OUTPUT = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                   'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                   'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})


def decode(msg):
    if msg.width == 0 or msg.height == 0:
        return np.empty(0, dtype=OUTPUT)
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.name in names or field.datatype not in TYPES or field.count < 1:
            raise ValueError('Invalid PointCloud2 field')
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + TYPES[field.datatype])
        if field.offset + dtype.itemsize * field.count > msg.point_step:
            raise ValueError('Field exceeds point_step')
        names.append(field.name)
        formats.append(dtype if field.count == 1 else (dtype, field.count))
        offsets.append(field.offset)
    if not {'x', 'y', 'z'}.issubset(names):
        raise ValueError('PointCloud2 requires x/y/z')
    if any(f.count != 1 for f in msg.fields if f.name in ['x', 'y', 'z']):
        raise ValueError('PointCloud2 coordinates must be scalar fields')
    if msg.row_step < msg.point_step * msg.width or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated PointCloud2 data or invalid row stride')
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets,
                      'itemsize': msg.point_step})
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step)).reshape(-1).copy()


def xt32_points(msg, sigma=0.0, dropout=0.0, rng=None):
    if not np.isfinite(sigma) or sigma < 0 or not 0 <= dropout <= 1:
        raise ValueError('Invalid noise/dropout parameter')
    rng = rng if rng is not None else np.random.default_rng()
    raw = decode(msg)
    xyz = np.column_stack([raw[k] for k in ('x', 'y', 'z')]).astype(float)
    ranges = np.linalg.norm(xyz, axis=1)
    valid = np.isfinite(xyz).all(axis=1) & (ranges >= 0.05) & (ranges <= 120)
    xyz, ranges, raw = xyz[valid], ranges[valid], raw[valid]
    elevation = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
    rings = np.rint(elevation + 16).astype(int)
    if np.any((rings < 0) | (rings >= 32) | (np.abs(elevation - (rings - 16)) > 0.15)):
        raise ValueError('Cloud does not match configured XT32 beam angles')
    output = np.zeros(len(xyz), dtype=OUTPUT)
    disturbed = ranges + rng.normal(0, sigma, len(ranges))
    xyz *= (disturbed / ranges)[:, None]
    for index, name in enumerate(('x', 'y', 'z')):
        output[name] = xyz[:, index]
    output['ring'] = rings
    output['intensity'] = raw['intensity'] if 'intensity' in raw.dtype.names else 1.0
    # Snapshot scans: all beams were acquired at the message time.
    output['time'] = 0.0
    keep = (disturbed >= 0.05) & (disturbed <= 120) & (rng.random(len(xyz)) >= dropout)
    return output[keep]


def encode(points, header):
    msg = PointCloud2(header=header, height=1, width=len(points), is_bigendian=False,
                      point_step=24, row_step=24 * len(points), is_dense=True)
    for name, datatype in [('x', 7), ('y', 7), ('z', 7), ('intensity', 7), ('ring', 4), ('time', 7)]:
        msg.fields.append(PointField(name=name, offset=OUTPUT.fields[name][1], datatype=datatype, count=1))
    # Humble validates bytes one element at a time; its uint8 array path avoids
    # that Python work for full scans while retaining exactly the same payload.
    data = array('B')
    data.frombytes(points.astype(OUTPUT, copy=False).tobytes())
    msg.data = data
    return msg
