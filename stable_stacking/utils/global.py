class Global:
    def __init__(self):
        self._pallet_discretization = 10
        self._states = []  # List of State objects

    # getters
    def get_pallet_discretization(self):
        return self._pallet_discretization

    def get_states(self):
        return self._states