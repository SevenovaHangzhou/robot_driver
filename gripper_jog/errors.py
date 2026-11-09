class JogError(ValueError):
    def __init__(self, code, message, *, details=None):
        self.code = code
        self.details = details
        super().__init__(message)
