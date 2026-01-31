@pytest.mark.skip(reason="This is an example test")
def test_hello():
    print("Hello, World! I'm a software test!")
    assert True # Some assertion here
    
@pytest.mark.skip(reason="This is an example hardware test")
@pytest.mark.hardware # Will use this to mark hardware tests
def test_example_hardware():
    print("Hello, World! I'm a hardware test!")
    assert True # Some assertion here
