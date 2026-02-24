from abc import ABC, abstractmethod
from dataclasses import dataclass, field, is_dataclass
from typing import List, override
from enum import IntEnum
import math
from packaging.version import Version, InvalidVersion
from tf_transformations import quaternion_from_euler

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from moveit_msgs.msg import CollisionObject as MoveitColObjMsg
from rosiris_manipulation_interfaces.msg import CollisionEntry as CollisionEntryMsg
from rosiris_manipulation_interfaces.msg import CollisionMatrixUpdate as CollisionMatrixUpdateMsg
from rosiris_manipulation_interfaces.msg import CollisionObject as RosirisColObjMsg
from shape_msgs.msg import Mesh as MeshMsg
from shape_msgs.msg import SolidPrimitive as SolidPrimitiveMsg
from std_msgs.msg import Header as HeaderMsg

from rosiris_manipulation_utils.mesh_loader import MeshLoader

class MsgClass(ABC):
    @abstractmethod
    def to_msg(self):
        pass

class SceneManagerOperation(IntEnum):
    ADD = 0
    REMOVE = 1
    APPEND = 2
    MOVE = 3


class SolidPrimitive(IntEnum):
    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4
    PRISM = 5


class CMUMode(IntEnum):
    REPLACE = 0
    MERGE = 1
    REMOVE = 2

class RotationUnit(IntEnum):
    DEG = 0
    RAD = 1
@dataclass
class Metadata:
    schema_version: str
    name: str = ""
    author: str = ""
    description: str = ""
    created: str = ""

    def __bool__(self) -> bool:
        try:
            Version(self.schema_version)
        except InvalidVersion:
            return False
        return True
    

@dataclass
class Position(MsgClass):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @override
    def to_msg(self) -> PointMsg:
        return PointMsg(x=self.x, y=self.y, z=self.z)


@dataclass
class OrientationRPY(MsgClass):
    r: float = 0.0
    p: float = 0.0
    y: float = 0.0
    unit: RotationUnit = RotationUnit.DEG

    @override
    def to_msg(self) -> QuaternionMsg:
        if self.unit == RotationUnit.DEG:
            r_rad = math.radians(self.r)
            p_rad = math.radians(self.p)
            y_rad = math.radians(self.y)
            qx, qy, qz, qw = quaternion_from_euler(r_rad, p_rad, y_rad)
        else:
            qx, qy, qz, qw = quaternion_from_euler(self.r, self.p, self.y)
        return QuaternionMsg(x=qx, y=qy, z=qz, w=qw)


@dataclass
class Pose(MsgClass):
    position: Position = field(default_factory=Position)
    orientation: OrientationRPY = field(default_factory=OrientationRPY)

    @override
    def to_msg(self) -> PoseMsg:
        msg = PoseMsg()
        msg.position = self.position.to_msg()
        msg.orientation = self.orientation.to_msg()
        return msg


@dataclass
class Stamp(MsgClass):
    sec: int = 0
    nanosec: int = 0

    @override
    def to_msg(self) -> TimeMsg:
        return TimeMsg(sec=self.sec, nanosec=self.nanosec)

@dataclass
class Header(MsgClass):
    frame_id: str
    stamp: Stamp = field(default_factory=Stamp)

    @override
    def to_msg(self) -> HeaderMsg:
        return HeaderMsg(frame_id=self.frame_id, stamp=self.stamp.to_msg())

@dataclass
class Primitive(MsgClass):
    type: SolidPrimitive
    dimensions: List[float]
    pose: Pose = field(default_factory=Pose)

    def to_msg(self):
        prim_msg = SolidPrimitiveMsg()
        prim_msg.type = int(self.type.value)
        prim_msg.dimensions = self.dimensions
        return prim_msg, self.pose.to_msg()


@dataclass
class Mesh(MsgClass):
    path: str
    pose: Pose = field(default_factory=Pose)

    @override
    def to_msg(self) -> tuple[MeshMsg, PoseMsg]:
        mesh_msg = MeshLoader.mesh_from_file(path=self.path)
        pose_msg = self.pose.to_msg()
        return mesh_msg,pose_msg

@dataclass
class Subframe(MsgClass):
    name: str
    pose: Pose = field(default_factory=Pose)
    is_grasp_point: bool = False

    @override
    def to_msg(self) -> tuple[str, PoseMsg]:
        pose_msg = self.pose.to_msg()
        return self.name,pose_msg

@dataclass
class CollisionObject(MsgClass):
    header: Header 
    id: str

    operation: SceneManagerOperation = SceneManagerOperation.ADD
    pose: Pose = field(default_factory=Pose)

    primitives: List[Primitive] = field(default_factory=list)
    meshes: List[Mesh] = field(default_factory=list)
    subframes: List[Subframe] = field(default_factory=list)

    self_collision_allowed: bool = True
    allowed_touch_links: List[str] = field(default_factory=list)

    # needed since operation is a byte and with IntEnum
    # we get a int even if we have valid value e.g. 0-4
    # we end up with some weird serialization error...
    _OPERATION_MAP = {
        SceneManagerOperation.ADD: MoveitColObjMsg.ADD,
        SceneManagerOperation.REMOVE: MoveitColObjMsg.REMOVE,
        SceneManagerOperation.APPEND: MoveitColObjMsg.APPEND,
        SceneManagerOperation.MOVE: MoveitColObjMsg.MOVE,
    }

    def to_moveit_msg(self) -> MoveitColObjMsg:
        msg = MoveitColObjMsg()

        # Header
        msg.header = self.header.to_msg()

        # Basic fields
        msg.id = self.id
        msg.operation = self._OPERATION_MAP[self.operation]
        msg.pose = self.pose.to_msg()

        # Primitives
        for primitive in self.primitives:
            prim_msg, prim_pose = primitive.to_msg()
            msg.primitives.append(prim_msg)
            msg.primitive_poses.append(prim_pose)

        # Meshes (not implemented)
        for mesh in self.meshes:
            mesh_msg, mesh_pose = mesh.to_msg()
            msg.meshes.append(mesh_msg)
            msg.mesh_poses.append(mesh_pose)

        # Subframes
        for sub in self.subframes:
            name, pose_msg = sub.to_msg()
            if name:
                msg.subframe_names.append(name or "")
                msg.subframe_poses.append(pose_msg)

        return msg

    @override
    def to_msg(self) -> RosirisColObjMsg:
        rosiris_msg = RosirisColObjMsg()
        rosiris_msg.collision_object = self.to_moveit_msg()
        rosiris_msg.self_collision_allowed = bool(self.self_collision_allowed)
        rosiris_msg.allowed_touch_links = self.allowed_touch_links
        return rosiris_msg

@dataclass
class CollisionMatrixEntry(MsgClass):
    touch_link: str
    collision_allowed: bool


    @override
    def to_msg(self) -> CollisionEntryMsg:
        return CollisionEntryMsg(touch_link=self.touch_link, collision_allowed=self.collision_allowed)

@dataclass
class CollisionMatrixUpdate(MsgClass):
    target_link: str
    mode: CMUMode
    collision_entries: List[CollisionMatrixEntry] = field(default_factory=list)

    @override
    def to_msg(self) -> CollisionMatrixUpdateMsg:
        cmu = CollisionMatrixUpdateMsg()
        cmu.target_link = self.target_link
        cmu.mode = int(self.mode.value)
        for col_entry in self.collision_entries:
            cmu.collision_entries.append(col_entry.to_msg())

@dataclass
class Scenario(MsgClass):
    metadata: Metadata
    collision_objects: List[CollisionObject] = field(default_factory=list)
    collision_matrix_updates: List[CollisionMatrixUpdate] = field(default_factory=list)

    @override
    def to_msg(self) -> tuple[List[RosirisColObjMsg], List[CollisionMatrixUpdateMsg]]:
        col_obj_msgs = []
        for col_obj in self.collision_objects:
            col_obj_msgs.append(col_obj.to_msg())

        col_mtrx_upds = []
        for col_entry in self.collision_matrix_updates:
            col_mtrx_upds.append(col_entry.to_msg())

        return col_obj_msgs, col_mtrx_upds

    def __bool__(self) -> bool:
        return bool(self.metadata)
    
    @property
    def name(self) -> str:
        return self.metadata.name
    
    @property
    def collision_object_ids(self) -> List[str]:
        return [col_obj.id for col_obj in self.collision_objects]