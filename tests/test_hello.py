@pytest.mark.skip(reason="This is an example test")
def test_hello():
    print("Hello, World!")
    assert True # Do nothing here
    
@pytest.mark.skip(reason="This is an example hardware test")
@pytest.mark.hardware
def test_example_hardware():
    assert True # Some assertion here

@pytest.mark.skip(reason="This is an example software test")
@pytest.mark.software
def test_example_hardware():
    assert True # Some assertion here
