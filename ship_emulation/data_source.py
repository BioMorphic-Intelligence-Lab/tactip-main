import csv
import json
import socket
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterator, Optional


@dataclass(frozen=True)
class ShipPose:
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def as_robot_pose(self):
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


class DataSource(ABC):
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    @abstractmethod
    def poses(self) -> Iterator[ShipPose]:
        pass

    @abstractmethod
    def close(self):
        pass


class CsvFileSource(DataSource):
    def __init__(self, path: str, skip_header: bool = True):
        self._path = path
        self._skip_header = skip_header

    def poses(self) -> Iterator[ShipPose]:
        with open(self._path, newline="") as f:
            reader = csv.reader(f)
            if self._skip_header:
                next(reader, None)
            for lineno, row in enumerate(reader, start=2 if self._skip_header else 1):
                if len(row) != 7:
                    raise ValueError(
                        f"{self._path}:{lineno}: expected 7 columns, got {len(row)}"
                    )
                try:
                    values = [float(v) for v in row]
                except ValueError as e:
                    raise ValueError(f"{self._path}:{lineno}: {e}") from e
                yield ShipPose(*values)

    def close(self):
        pass


class UdpSource(DataSource):
    def __init__(
        self,
        host: str = "",
        port: int = 5005,
        timeout: float = 5.0,
        max_poses: Optional[int] = None,
    ):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._max_poses = max_poses
        self._sock: Optional[socket.socket] = None

    def _ensure_socket(self):
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self._host, self._port))
            self._sock.settimeout(self._timeout)

    def poses(self) -> Iterator[ShipPose]:
        self._ensure_socket()
        count = 0
        while self._max_poses is None or count < self._max_poses:
            data, _ = self._sock.recvfrom(4096)
            d = json.loads(data.decode())
            yield ShipPose(
                timestamp=float(d["timestamp"]),
                x=float(d["x"]),
                y=float(d["y"]),
                z=float(d["z"]),
                roll=float(d["roll"]),
                pitch=float(d["pitch"]),
                yaw=float(d["yaw"]),
            )
            count += 1

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None
