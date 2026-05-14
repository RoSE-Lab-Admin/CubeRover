from abc import ABC, abstractmethod

class BaseStep(ABC):
    @abstractmethod
    async def execute(self, step_config: dict):
        """Runs the step logic."""
        pass

    @abstractmethod
    async def stop(self):
        """Immediately halts the step (safestate)."""
        pass

class BasePassCriteria(ABC):
    @abstractmethod
    def evaluate(self, criteria_config: dict, test_data) -> dict:
        """
        Evaluates the data against the config.
        Should return a dictionary with at least {'passed': bool, 'details': str}
        """
        pass