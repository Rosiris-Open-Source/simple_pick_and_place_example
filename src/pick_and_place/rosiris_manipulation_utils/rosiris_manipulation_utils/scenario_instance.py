from typing import List

from rosiris_manipulation_interfaces.msg import CollisionMatrixUpdate as CollisionMatrixUpdateMsg
from rosiris_manipulation_interfaces.msg import CollisionObject as RosirisColObjMsg

from rosiris_manipulation_utils.scenario_loader import ScenarioLoader, NoSuitableLoaderError, LOADER_REGISTRY
from rosiris_manipulation_utils.utilities import get_file_type

class ScenarioInstance():
    def __init__(self, path: str):
        self._scenario_loader = self._select_loader(path)
        self._scenario = self._scenario_loader.load_scenario(path)
        self._loaded_in_scene = False

    def _select_loader(self, path: str) -> ScenarioLoader:
        file_type = get_file_type(path)
        for loader_cls in LOADER_REGISTRY:
            if file_type in loader_cls.SUPPORTED_FILES:
                return loader_cls()

        raise NoSuitableLoaderError(
            f"No loader registered for file type {file_type}"
        )

    # delegate all attribute access to the underlying scenario
    # e.g. instance.name -> returns instance._scenario.name
    def __getattr__(self, item):
        return getattr(self._scenario, item)

    @property
    def loaded_in_scene(self) -> bool:
        return self._loaded_in_scene

    @loaded_in_scene.setter
    def loaded_in_scene(self, value: bool):
        if not isinstance(value, bool):
            raise ValueError("loaded must be a boolean")
        self._loaded_in_scene = value
