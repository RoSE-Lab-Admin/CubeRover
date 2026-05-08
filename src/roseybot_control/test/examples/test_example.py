import pytest

# Note: This test would need to be in the "software" folder to be tested with `colcon test`.
@pytest.mark.skip(reason="This is an example test")
def test_hello():
    print("Hello, World! I'm a software test!")
    assert True # Some assertion here
    
# Note: This test would need to be in the "hardware" folder to be tested with `colcon test`.
@pytest.mark.skip(reason="This is an example hardware test")
@pytest.mark.hardware # Marks a hardware test for pytest.
def test_example_hardware():
    print("Hello, World! I'm a hardware test!")
    assert True # Some assertion here
