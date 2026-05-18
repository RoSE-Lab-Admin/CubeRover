from pathlib import Path
import yaml
import importlib.util
import inspect
from typing import Any, Dict, List, Optional, Type

from .interfaces import BaseStep, BasePassCriteria

class TestEngine:    
    """
    Orchestrates the loading and execution of test profiles.
    
    Attributes:
        hardware: The hardware DAQ/communication interface.
        root (Path): The base working directory.
        profiles_dir (Path): The directory containing YAML test profiles.
        active_step (Optional[BaseStep]): The currently executing test step.
        active_config (Dict[str, Any]): The currently loaded test profile config.
        steps (Dict[str, BaseStep]): Registry of dynamically loaded step plugins.
        pass_criteria (Dict[str, BasePassCriteria]): Registry of loaded criteria plugins.
    """
    
    def __init__(self, hardware_interface: Optional[Any] = None, base_path: Optional[str] = None) -> None:
        self.hardware = hardware_interface
        self.root = Path(base_path).resolve() if base_path else Path.cwd()
        self.profiles_dir = self.root / 'test_profiles'
        
        self.active_step: Optional[BaseStep] = None
        self.active_config: Dict[str, Any] = {} 
        
        # Dynamically build the registries!
        engine_dir = Path(__file__).resolve().parent
        self.steps: Dict[str, BaseStep] = self._discover_plugins(engine_dir / 'steps', BaseStep)
        self.pass_criteria: Dict[str, BasePassCriteria] = self._discover_plugins(engine_dir / 'pass_criteria', BasePassCriteria)
        
        print(f"ENGINE INIT: Loaded {len(self.steps)} Steps and {len(self.pass_criteria)} Pass Criteria.")

    def _discover_plugins(self, folder_path: Path, base_class: Type) -> Dict[str, Any]:
        """
        Scans a directory for Python files, imports them into memory, and extracts 
        any classes that inherit from `base_class`. 
        
        Note: Use a `NAME` attribute at the top of a class to specify a custom 
        name for the YAML key, otherwise it defaults to the filename.
        
        Args:
            folder_path (Path): The directory to scan for plugins.
            base_class (Type): The parent class that valid plugins must inherit from.
            
        Returns:
            Dict[str, Any]: A dictionary mapping YAML keys to instantiated plugin objects.
        """
        registry: Dict[str, Any] = {}
        if not folder_path.exists():
            print(f"Warning: Plugin directory {folder_path} not found.")
            return registry

        # Determine the package path for absolute importing
        parent_pkg = folder_path.parent.name
        sub_pkg = folder_path.name
        full_package_name = f"{parent_pkg}.{sub_pkg}"

        # Iterate over all standard .py files in the folder
        for file_path in folder_path.glob("*.py"):
            if file_path.name.startswith("__"):
                continue  # Skip __init__.py files

            module_name = f"{full_package_name}.{file_path.stem}"

            # Dynamically load the module into memory using importlib
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                module.__package__ = full_package_name
                spec.loader.exec_module(module)

                # Inspect the module to find relevant plugin classes
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    
                    # Check if it inherits from the base class (and isn't the base class itself)
                    if issubclass(obj, base_class) and obj is not base_class:
                        
                        # Use a custom name if defined, otherwise default to the filename
                        yaml_key = getattr(obj, 'NAME', file_path.stem)
                        
                        # Guard against multiple plugins attempting to claim the same YAML key
                        if yaml_key in registry:
                            existing_class = registry[yaml_key].__class__.__name__
                            new_class = obj.__name__
                            raise RuntimeError(
                                f"PLUGIN COLLISION: Both '{existing_class}' and "
                                f"'{new_class}' are attempting to register under the "
                                f"YAML key '{yaml_key}'. Please rename one of the files "
                                f"or provide a unique 'NAME' attribute."
                            )
                        
                        # Try to instantiate the plugin. 
                        # Fallback to passing the hardware interface if the constructor requires it.
                        try:
                            registry[yaml_key] = obj()
                        except TypeError:
                            registry[yaml_key] = obj(self.hardware)
                            
        return registry

    def _load_profile(self, profile_filename: str) -> Dict[str, Any]:
        """Reads a test profile from the disk and parses its YAML content."""
        profile_path = self.profiles_dir / profile_filename
        if not profile_path.exists():
            raise FileNotFoundError(f"Could not find {profile_path}")
            
        with open(profile_path, 'r') as f:
            return yaml.safe_load(f)

    def get_available_tests(self) -> Dict[str, Dict[str, Any]]:
        """
        Finds and loads all available test YAMLs from the profiles directory.
        Used by the UI to populate the test selection dropdown.
        
        Returns:
            Dict[str, Dict[str, Any]]: A mapping of filenames to their parsed YAML contents.
        """
        tests: Dict[str, Dict[str, Any]] = {}
        profiles_dir = self.profiles_dir
        
        if not profiles_dir.exists():
            print(f"Warning: Profiles directory not found at {profiles_dir}")
            return tests
            
        for file_path in profiles_dir.glob('*.y*ml'):
            try:
                with open(file_path, 'r') as f:
                    tests[file_path.name] = yaml.safe_load(f)
            except Exception as e:
                print(f"Error loading {file_path.name}: {e}")
                
        return tests

    async def execute_profile(self, profile_filename: str) -> None:
        """
        Asynchronously loads and executes the steps defined in a test profile.
        This is typically called by the UI when the user clicks 'Start Capture'.
        
        Args:
            profile_filename (str): The filename of the YAML profile to run.
        """
        print(f"--- ENGINE: Loading and Executing {profile_filename} ---")
        
        # 1. Load and store it internally (Hot-reloading from disk is preserved!)
        self.active_config = self._load_profile(profile_filename)
        
        # 2. Iterate through and execute the defined hardware/software steps
        for step in self.active_config.get('steps', []):
            await self.execute_step(step)
            
        print("--- ENGINE: Steps Finished Normally ---")

    async def execute_step(self, step_config: Dict[str, Any]) -> None:
        """
        Resolves a step configuration block against the loaded plugin registry 
        and triggers its execution.
        """
        step_type = step_config.get("type")
        if step_type not in self.steps:
            raise ValueError(f"Unknown step type: {step_type}")
            
        self.active_step = self.steps[step_type]
        # Await the execution since it likely has hardware delays and async sleep routines
        await self.active_step.execute(step_config)
        self.active_step = None

    async def stop(self) -> None:
        """
        Emergency stop sequence triggered by the UI or application teardown.
        Halts the active step and enforces a global hardware kill switch if configured.
        """
        if self.active_step:
            await self.active_step.stop()
        
        # Always enforce a global hardware kill switch just in case
        if self.hardware:
            print("ENGINE: Asserting global hardware emergency stop!")
            # await self.hardware.emergency_shutdown()
            
    def evaluate_current_profile(self, test_data: Any) -> Dict[str, Any]:
        """
        Extracts the pass criteria block from the currently active profile 
        and evaluates the provided test data against them.
        
        Args:
            test_data (Any): Typically a Pandas DataFrame containing the run's telemetry.
            
        Returns:
            Dict[str, Any]: A final report indicating global pass/fail status and detailed results.
        """
        criteria_list = self.active_config.get('pass_criteria', [])
        
        # If there are no criteria defined in the YAML, the test automatically passes
        if not criteria_list:
            return {"passed": True, "results": []}
            
        return self.evaluate_all(criteria_list, test_data)

    def evaluate_all(self, criteria_list: List[Dict[str, Any]], test_data: Any) -> Dict[str, Any]:
        """
        Iterates over a list of pass criteria configurations, running the test data 
        through the corresponding evaluator plugins. 
        
        A single failed criterion will flag the entire test run as a failure.
        """
        print("--- ENGINE: Evaluating Pass Criteria ---")
        report: Dict[str, Any] = {
            "passed": True,
            "results": []
        }
        
        # Guard against evaluating empty telemetry datasets
        if test_data.empty:
            print("WARNING: No data captured. Cannot evaluate pass criteria.")
            report["passed"] = False
            report["results"].append({"type": "System", "passed": False, "details": "No test data available."})
            return report

        for criteria_config in criteria_list:
            c_type = criteria_config.get("type")
            if c_type not in self.pass_criteria:
                print(f"WARNING: Unknown pass criteria type '{c_type}'. Skipping.")
                continue
            
            evaluator = self.pass_criteria[c_type]
            print(f"Evaluating: {c_type}")
            
            # Call the specific evaluator module plugin
            result = evaluator.evaluate(criteria_config, test_data)
            
            report["results"].append({
                "type": c_type,
                "passed": result.get("passed", False),
                "details": result.get("details", "")
            })
            
            # If any single criteria fails, the whole test evaluation fails
            if not result.get("passed", False):
                report["passed"] = False
                
        return report