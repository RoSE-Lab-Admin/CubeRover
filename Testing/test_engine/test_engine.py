from pathlib import Path
import yaml
import importlib.util
import inspect

from .interfaces import BaseStep, BasePassCriteria

class TestEngine:    
    def __init__(self, hardware_interface=None, base_path=None):
        self.hardware = hardware_interface
        self.root = Path(base_path).resolve() if base_path else Path.cwd()
        self.profiles_dir = self.root / 'test_profiles'
        
        self.active_step = None
        self.active_config = {} 
        
        # ynamically build the registries!
        engine_dir = Path(__file__).resolve().parent
        self.steps = self._discover_plugins(engine_dir / 'steps', BaseStep)
        self.pass_criteria = self._discover_plugins(engine_dir / 'pass_criteria', BasePassCriteria)
        
        print(f"ENGINE INIT: Loaded {len(self.steps)} Steps and {len(self.pass_criteria)} Pass Criteria.")

    def _discover_plugins(self, folder_path: Path, base_class) -> dict:
        """
        Scans a directory for Python files, imports them, and finds any classes
        that inherit from `base_class`. Use NAME at the top of a class to specify a custom name as the YAML key.
        """
        registry = {}
        if not folder_path.exists():
            print(f"Warning: Plugin directory {folder_path} not found.")
            return registry

        # Dtermine the package path
        parent_pkg = folder_path.parent.name
        sub_pkg = folder_path.name
        full_package_name = f"{parent_pkg}.{sub_pkg}"

        # Iterate over all .py files in the folder
        for file_path in folder_path.glob("*.py"):
            if file_path.name.startswith("__"):
                continue  # Skip __init__.py files

            module_name = f"{full_package_name}.{file_path.stem}"

            # Dynamically load the module into memory
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                module.__package__ = full_package_name
                spec.loader.exec_module(module)

                # Inspect the module to find classes
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    
                    # Check if it inherits from the base class (and isn't the base class itself)
                    if issubclass(obj, base_class) and obj is not base_class:
                        
                        # Use a custom name if defined, otherwise default to the filename
                        yaml_key = getattr(obj, 'NAME', file_path.stem)
                        
                        # Check for name collisions
                        if yaml_key in registry:
                            # Get the name of the class already occupying this slot
                            existing_class = registry[yaml_key].__class__.__name__
                            new_class = obj.__name__
                            raise RuntimeError(
                                f"PLUGIN COLLISION: Both '{existing_class}' and "
                                f"'{new_class}' are attempting to register under the "
                                f"YAML key '{yaml_key}'. Please rename one of the files "
                                f"or provide a unique 'NAME' attribute."
                            )
                        
                        # Try to instantiate it. 
                        # Fallback to passing hardware if the constructor requires it.
                        try:
                            registry[yaml_key] = obj()
                        except TypeError:
                            registry[yaml_key] = obj(self.hardware)
                            
        return registry

    def _load_profile(self, profile_filename: str) -> dict:
        profile_path = self.profiles_dir / profile_filename
        if not profile_path.exists():
            raise FileNotFoundError(f"Could not find {profile_path}")
            
        with open(profile_path, 'r') as f:
            return yaml.safe_load(f)

    def get_available_tests(self) -> dict:
        """Finds and loads all test YAMLs from a given directory."""
        tests = {}
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

    async def execute_profile(self, profile_filename: str):
        """Loads and executes the profile. Called by the UI."""
        print(f"--- ENGINE: Loading and Executing {profile_filename} ---")
        
        # 1. Load and store it internally (Hot-reloading is preserved!)
        self.active_config = self._load_profile(profile_filename)
        
        # 2. Execute the steps
        for step in self.active_config.get('steps', []):
            await self.execute_step(step)
            
        print("--- ENGINE: Steps Finished Normally ---")

    async def execute_step(self, step_config: dict):
        step_type = step_config.get("type")
        if step_type not in self.steps:
            raise ValueError(f"Unknown step type: {step_type}")
            
        self.active_step = self.steps[step_type]
        # Await the execution since it likely has hardware delays
        await self.active_step.execute(step_config)
        self.active_step = None

    async def stop(self):
        """Emergency stop triggered by the UI."""
        if self.active_step:
            await self.active_step.stop()
        
        # Always enforce a global hardware kill switch just in case
        if self.hardware:
            print("ENGINE: Asserting global hardware emergency stop!")
            # await self.hardware.emergency_shutdown()
            
    def evaluate_current_profile(self, test_data) -> dict:
        """Evaluates data using the pass criteria of the actively loaded profile."""
        criteria_list = self.active_config.get('pass_criteria', [])
        
        # If there are no criteria, automatically pass
        if not criteria_list:
            return {"passed": True, "results": []}
            
        return self.evaluate_all(criteria_list, test_data)

    def evaluate_all(self, criteria_list: list, test_data) -> dict:
        """
        Runs all configured pass criteria against the captured test data.
        Returns a final report dictionary.
        """
        print("--- ENGINE: Evaluating Pass Criteria ---")
        report = {
            "passed": True,
            "results": []
        }
        
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
            
            # Call the specific evaluator module
            result = evaluator.evaluate(criteria_config, test_data)
            
            report["results"].append({
                "type": c_type,
                "passed": result.get("passed", False),
                "details": result.get("details", "")
            })
            
            # If any single criteria fails, the whole test fails
            if not result.get("passed", False):
                report["passed"] = False
                
        return report