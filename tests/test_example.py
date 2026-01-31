@pytest.mark.skip(reason="This is an example test")
def test_hello():
    print("Hello, World!")
    assert True # Some assertion here
    
@pytest.mark.skip(reason="This is an example hardware test")
@pytest.mark.hardware # Will use this to mark hardware tests
def test_example_hardware():
    assert True # Some assertion here

@pytest.mark.skip(reason="This is an example software test")
@pytest.mark.software # Will use this to mark software tests
def test_example_software():
    assert True # Some assertion here
