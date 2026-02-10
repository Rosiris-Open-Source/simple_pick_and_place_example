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
import pyassimp

from pathlib import Path

from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle

from rosiris_manipulation_utils.utilities import resolve_resource_path

class MeshLoader:  


    @staticmethod
    def mesh_from_file(*, path: str| Path, base_dir: str | None = None, scale=(1.0, 1.0, 1.0)) -> Mesh:
        """
        Load mesh file (STL, DAE, OBJ, etc.) into shape_msgs/msg/Mesh
        using pyassimp 
        """
        mesh_uri = resolve_resource_path(uri=path, base_dir=base_dir)

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
