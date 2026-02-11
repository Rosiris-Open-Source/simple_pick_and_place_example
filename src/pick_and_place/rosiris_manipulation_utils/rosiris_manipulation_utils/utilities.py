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

from urllib.parse import urlparse
from pathlib import Path
from ament_index_python import get_package_share_directory
from typing import Iterable


def resolve_resource_path(*, uri: str, base_dir: str | None = None, file_types: str | Iterable[str] | None = None) -> Path:
    """
    Resolve paths of type:
    - absolute paths
    - relative paths
    - package:// URIs

    :param uri: path or URI to resource
    :param base_dir: optional base directory for relative paths
    :param file_types: if file types are given, check if the resolved resource matches any in file_types
    :return: absolute filesystem path to the resource
    """
    abs_res : str
    # package:// URI
    if uri.startswith("package://"):
        parsed = urlparse(uri)
        pkg_name = parsed.netloc
        rel_path = parsed.path.lstrip("/")

        pkg_share = get_package_share_directory(pkg_name)
        abs_res = os.path.join(pkg_share, rel_path) 
    # Absolute path
    elif os.path.isabs(uri):
        abs_res = uri
    # Try relative path to file
    elif not base_dir or not base_dir.strip():
        base_dir = os.getcwd()
        abs_res = os.path.abspath(os.path.join(base_dir, uri))
    # Try relative path base_dir
    else:
        abs_res = os.path.abspath(os.path.join(base_dir, uri))
    
    abs_res_path = Path(abs_res)
    
    check_file_exists(abs_res_path)
    
    if file_types:
        check_file_type(abs_res_path, file_types)


    return abs_res_path


def check_file_exists(abs_res_path: Path) -> None:
    """
    Check if the file exists at the given path

    :param abs_res_path: path to the resource
    :return: None, raise FileNotFoundError on file not exist
    """
    if not abs_res_path.exists() or not abs_res_path.is_file():
        raise FileNotFoundError(abs_res_path)
    
def get_file_type(abs_res_path: str | Path) -> str:
    """
    Return the type of a passed path
    
    :param abs_res_path: Path to the file
    :return: type of file e.g. .yaml for file.yaml, archive.tar.gz for file.archive.tar.gz
    """
    abs_res_path = Path(abs_res_path)
    return "".join(abs_res_path.suffixes).lower()

def check_file_type(abs_res_path: Path, file_types: str | Iterable[str]) -> None:
    """
    Check if the abs_res_path resolves to the expected file type given by file_types

    :param abs_res_path: path to resource 
    :param file_types: check if the resolved resource matches any given in file_types
    :return: None, raise ValueError on file not of file_types
    """
    if isinstance(file_types, str):
        file_types = (file_types,)
    else:
        file_types = tuple(file_types)

    normalized = tuple(
        ft.lower() if ft.startswith(".") else f".{ft.lower()}"
        for ft in file_types
    )

    if abs_res_path.suffix.lower() not in normalized:
        raise ValueError(
            f"{abs_res_path} must be one of {normalized}"
        )
