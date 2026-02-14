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
import yaml

from abc import ABC, abstractmethod
from dataclasses import fields, is_dataclass
from enum import IntEnum
from packaging.version import Version
from typing import override, Type, TypeVar, Any, Dict, Generic, ClassVar

from rosiris_manipulation_utils.scenario_models import Scenario
from rosiris_manipulation_utils.utilities import resolve_resource_path

class LoadScenarioError(Exception):
    """Raised when a scenario cannot be loaded or validated."""
    pass

class NoSuitableLoaderError(ValueError):
    """Raised when there is no Loader for the specified file type"""
    pass

class ScenarioLoader(ABC):
    SUPPORTED_FILES: ClassVar[tuple[str, ...]]
    
    @abstractmethod
    def load_scenario(self, path: str) -> Scenario:
        pass

    @property
    @abstractmethod
    def CURRENT_SCHEMA_VERSION(self) -> Version:
        pass

    @property
    @abstractmethod
    def MIN_SUPPORTED_VERSION(self) -> Version:
        pass


T = TypeVar("T")
class YamlScenarioLoader(ScenarioLoader, Generic[T]):
    SUPPORTED_FILES: ClassVar[tuple[str, ...]] = (".yaml", ".yml", ".json")

    @override
    def load_scenario(self, path: str) -> Scenario:
        """
        Loads the scenario at path.
        
        :param path: path to the yaml file defining the scenario
        :return: Scenario as a Scenario dataclass.
            Raises:
                ValueError: If scenario not valid.
        """

        try:
            resolved_path = resolve_resource_path(uri=path, file_types=self.SUPPORTED_FILES)
            with open(resolved_path, "r", encoding="utf-8") as f:
                data : dict = yaml.safe_load(f)
        except FileNotFoundError as e:
            raise LoadScenarioError(f"Resource:{path} not resolvable.") from e
        except UnicodeDecodeError as e:
            raise LoadScenarioError(f"File is not valid UTF-8 text: {path}.{e}") from e
        except ValueError as e:
            raise LoadScenarioError(f"Not a yaml file:{path}. {e}") from e
        except yaml.YAMLError as e:
            raise LoadScenarioError(f"Failed to parse YAML: {path}. {e}") from e
        
        self._check_schema_version_is_supported(data)

        return self._parse_yaml_recursive(Scenario, data)
   
    @property
    @override
    def CURRENT_SCHEMA_VERSION(self) -> Version:
        return Version("1.0.0")

    @property
    @override
    def MIN_SUPPORTED_VERSION(self) -> Version:
        return Version("1.0.0")


    def _parse_yaml_recursive(self, cls: Type[T], data: Dict[str, Any]) -> T:
        """
        Recursively parse a dictionary into a dataclass, converting strings to enums where needed.
        """
        if not is_dataclass(cls):
            raise TypeError(f"{cls} is not a dataclass")

        init_kwargs = {}
        for f in fields(cls):
            value = data.get(f.name)

            if value is None:
                continue  # keep default

            # Handle enums
            if isinstance(f.type, type) and issubclass(f.type, IntEnum):
                init_kwargs[f.name] = self._normalize_enum_input(f.type, value)
            # Lists
            elif getattr(f.type, "__origin__", None) is list:
                inner_type = f.type.__args__[0]
                if not isinstance(value, list):
                    # Robustness: wrap single item if user inputs
                    # value instead of [value] we fallback to list
                    value = [value] 

                if is_dataclass(inner_type): # list if dataclass
                    init_kwargs[f.name] = [self._parse_yaml_recursive(inner_type, v) for v in value]
                elif isinstance(inner_type, type) and issubclass(inner_type, IntEnum): # list of enums
                    init_kwargs[f.name] = [self._normalize_enum_input(inner_type, v) for v in value]
                else: # list of primitives
                    init_kwargs[f.name] = value

            # Handle nested dataclasses
            elif is_dataclass(f.type):
                init_kwargs[f.name] = self._parse_yaml_recursive(f.type, value)
            else:
                init_kwargs[f.name] = value

        return cls(**init_kwargs)
    
    def _normalize_enum_input(self, enum_cls: Type[IntEnum], value: Any) -> IntEnum:  
        if isinstance(value, str):
            # normalize fuzzy input like "value" or " Value" or, ...
            # to VALUE
            normalized = value.strip().upper()
            try:
                return enum_cls[normalized]
            except KeyError:
                valid = [e.name for e in enum_cls]
                raise ValueError(f"Invalid string '{value}' for {enum_cls.__name__}. Expected {valid}")
        else:
            # Handle raw integers
            return enum_cls(value)
    
    def _check_schema_version_is_supported(self, data: dict) -> None:
        metadata = data.get("metadata")
        if metadata is None:
            raise ValueError("Metadata section not found in YAML")

        version_str = str(metadata.get("schema_version", ""))
        if not version_str:
            raise ValueError("schema_version not found in metadata")

        if not self._schema_version_supported(version_str):
            raise ValueError(f"Unsupported schema version: {version_str}")

    def _schema_version_supported(self, version: str) -> bool:
        version = Version(version)
        if version < self.MIN_SUPPORTED_VERSION:
            return False
        if version > self.CURRENT_SCHEMA_VERSION:
            return False
        return True
    

LOADER_REGISTRY: tuple[type[ScenarioLoader], ...] = (
    YamlScenarioLoader,
)