from typing import Dict, TypedDict

from .interfaces import BaseStep, BasePassCriteria

class TestProfile(TypedDict):
    name: str
    description: str
    steps: Dict[str, BaseStep]
    pass_criteria: Dict[str, BasePassCriteria]