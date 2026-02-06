# Copyright 2026 Manuel Muth
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import pyassimp

from urllib.parse import urlparse
from pathlib import Path
from ament_index_python import get_package_share_directory

from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle

class MeshLoader:  

    @staticmethod
    def resolve_mesh_path(*, uri: str, base_dir: str | None = None) -> Path:
        """
        Resolve paths of type:
        - absolute paths
        - relative paths
        - package:// URIs

        :param uri: mesh path or URI
        :param base_dir: optional base directory for relative paths
        :return: absolute filesystem path
        """

        # 1) package:// URI
        if uri.startswith("package://"):
            parsed = urlparse(uri)
            pkg_name = parsed.netloc
            rel_path = parsed.path.lstrip("/")

            pkg_share = get_package_share_directory(pkg_name)
            abs_path = os.path.join(pkg_share, rel_path)

            if not os.path.exists(abs_path):
                raise FileNotFoundError(abs_path)

            return Path(abs_path)

        # 2) Absolute path
        if os.path.isabs(uri):
            abs_path = uri
            if not os.path.exists(abs_path):
                raise FileNotFoundError(abs_path)
            return Path(abs_path)

        # 3) Relative path
        if not base_dir or not base_dir.strip():
            base_dir = os.getcwd()

        abs_path = os.path.abspath(os.path.join(base_dir, uri))

        if not os.path.exists(abs_path):
            raise FileNotFoundError(abs_path)

        return Path(abs_path)

    @staticmethod
    def mesh_from_file(*, path: str| Path, base_dir: str | None = None, scale=(1.0, 1.0, 1.0)) -> Mesh:
        """
        Load mesh file (STL, DAE, OBJ, etc.) into shape_msgs/msg/Mesh
        using pyassimp 
        """
        mesh_uri = MeshLoader.resolve_mesh_path(uri=path, base_dir=base_dir)

        with pyassimp.load(str(mesh_uri)) as scene:
            if not scene.meshes:
                raise RuntimeError("No meshes found in file")

            ros_mesh = Mesh()
            mesh = scene.meshes[0]  # take first mesh

            # vertices
            for v in mesh.vertices:
                p = Point()
                p.x = float(v[0] * scale[0])
                p.y = float(v[1] * scale[1])
                p.z = float(v[2] * scale[2])
                ros_mesh.vertices.append(p)

            # faces
            for face in mesh.faces:
                if len(face) != 3:
                    continue  # skip non-triangle faces
                tri = MeshTriangle()
                tri.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
                ros_mesh.triangles.append(tri)

            return ros_mesh
