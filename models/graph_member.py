class GraphMember:
    """The base class for all objects that could be displayed in the 3D panel of Foxglove

    Attributes:
        frame_name      The string name of the frame that this object represents. (i.e. "base_link")
        name            The string internal name of this object. (i.e. "Chimera D4")
        parent_frame    The string name of the frame of the parent to this object. (i.e. "map")
    """
    frame_name:str
    name:str
    parent_frame:str
    
    # Constructors
    def __init__(self, name:str = None, frame_name:str = None, parent_frame:str = None):
        self.frame_name = frame_name
        self.name = name
        self.parent_frame = parent_frame

    def __str__(self):
        return f"{self.name}\nTransform: {self.parent_frame} -> {self.frame_name}\n"
