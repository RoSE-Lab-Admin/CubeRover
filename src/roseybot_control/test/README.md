# Unit Testing

Unit tests can be written using PyTest and GTest.

## Colcon Test

* `colcon test` is the primary command used to run both the Python and C++ tests. Please see the project root-level `README.md` for test commands.
* For tests to run, the tests must be located in the `hardware` or `software` folders depending on what is being tested.


## PyTest Test

* Pytest can also be used to only run the python tests.
* To be able to filter the Python tests when using `pytest - m "not hardware"`, you will also need to add markers to the hardware python tests. See the "examples/test_example.py" file for an example of how to mark python tests.
