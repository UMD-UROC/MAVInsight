class GraphMember:
    """The base class for all objects that could be displayed in the 3D panel of Foxglove"""
    frame_tf:str
    name:str
    
    # Constructors
    def __init__(self, name:str = None, frame_tf:str = None):
        self.name = name
        self.frame_tf = frame_tf
