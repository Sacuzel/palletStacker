class Global:
    def __init__(self):
        self._pallette_discretization = 10
        self._states = []  # List of State objects

    # getters
    def get_pallette_discretization(self):
        return self._pallette_discretization

    def get_states(self):
        return self._states