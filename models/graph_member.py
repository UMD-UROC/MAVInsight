class GraphMember:
    """The base class for all objects that could be displayed in the 3D panel of Foxglove

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `GraphMember` would need
        to be considered "valid".
        This attribute is inherited by subclasses of `GraphMember`. Each subclass will likely accumulate
        these attributes.

    Attributes
    ----------
    frame_name : str
        The string name of the frame that this object represents. (i.e. "base_link")
    name : str
        The string internal name of this object. (i.e. "Chimera D4")
    parent_frame : str
        The string name of the frame of the parent to this object. (i.e. "map")
    """
    frame_name:str
    name:str
    parent_frame:str
    tab_char:str

    param_reqs:list[str] = ["frame_name", "name", "parent_frame"]

    # Constructors
    def __init__(self, name:str=None, frame_name:str=None, parent_frame:str=None):
        self.frame_name = frame_name
        self.name = name
        self.parent_frame = parent_frame
        self.tab_char = "|   "

    def check_dict(self, config_params:dict):
        if not set(GraphMember.param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params in dict to create GraphMember. Must have all of: {GraphMember.param_reqs}")

        self.name = config_params["name"]
        self.frame_name = config_params["frame_name"]
        self.parent_frame=config_params["parent_frame"]

    @classmethod
    def from_dict(clazz, config_params:dict):
        g_member = clazz()

        g_member.check_dict(config_params)

        return g_member

    def __str__(self):
        return f"{self.name}\nTransform: {self.parent_frame} -> {self.frame_name}\n"
