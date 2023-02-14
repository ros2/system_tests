def pytest_addoption(parser):
    parser.addoption("--ros-workspaces", action="store")
    parser.addoption("--rmws", action="store")

def pytest_generate_tests(metafunc):
    if 'rmw' in metafunc.fixturenames:
        metafunc.parametrize("rmw", metafunc.config.option.rmws.split(':'))