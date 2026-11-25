import struct
from dataclasses import dataclass
from typing import Optional, Union

# Precompiled Little-Endian C-Structs
IMU_V1_STRUCT = struct.Struct("<qB6f")        # 33 bytes
IMU_V2_STRUCT = struct.Struct("<qqBB6f")      # 42 bytes
IMU_V3_STRUCT = struct.Struct("<qB10f")       # 49 bytes (Universal: Quat + Gyro + Accel)
GPS_STRUCT = struct.Struct("<q3d4f")          # 48 bytes
CLOCK_QUERY_STRUCT = struct.Struct("<qqqi")   # 28 bytes
CLOCK_REPLY_STRUCT = struct.Struct("<qqqqqq") # 48 bytes
CLOCK_REPORT_STRUCT = struct.Struct("<qqqq")  # 32 bytes


@dataclass(slots=True)
class ImuSample:
    ts_gyro_ns: int
    ts_accel_ns: int
    mode: int          # 0 = FUSED (Quaternion), 1 = RAW (Linear Accel + Gyro)
    flags: int
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0


@dataclass(slots=True)
class GpsSample:
    ts_phone_ns: int
    latitude: float
    longitude: float
    altitude: float
    speed: float
    accuracy: float
    bearing: float
    bearing_accuracy: float


def unpack_imu(payload: bytes) -> Optional[ImuSample]:
    n = len(payload)
    if n == IMU_V3_STRUCT.size:
        ts, mode, qx, qy, qz, qw, gx, gy, gz, ax, ay, az = IMU_V3_STRUCT.unpack(payload)
        return ImuSample(
            ts_gyro_ns=ts, ts_accel_ns=ts, mode=mode, flags=0,
            qx=qx, qy=qy, qz=qz, qw=qw,
            gx=gx, gy=gy, gz=gz,
            ax=ax, ay=ay, az=az
        )
    elif n == IMU_V2_STRUCT.size:
        ts_g, ts_a, mode, flags, v0, v1, v2, v3, v4, v5 = IMU_V2_STRUCT.unpack(payload)
    elif n == IMU_V1_STRUCT.size:
        ts_g, mode, v0, v1, v2, v3, v4, v5 = IMU_V1_STRUCT.unpack(payload)
        ts_a, flags = ts_g, 0
    else:
        return None

    if mode == 0:
        return ImuSample(ts_g, ts_a, mode, flags, qx=v0, qy=v1, qz=v2, qw=v3)
    return ImuSample(ts_g, ts_a, mode, flags, ax=v0, ay=v1, az=v2, gx=v3, gy=v4, gz=v5)


def unpack_gps(payload: bytes) -> Optional[GpsSample]:
    if len(payload) != GPS_STRUCT.size:
        return None
    return GpsSample(*GPS_STRUCT.unpack(payload))
