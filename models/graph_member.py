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

    @classmethod
    def from_dict(clazz, config_params:dict):
        # check for required Graph Member params
        if not clazz._dict_meets_reqs:
            raise ValueError(f"Not enough params in dict to create GraphMember. Must have all of: {clazz.param_reqs}")

        return clazz(name=config_params["name"], frame_name=config_params["frame_name"], parent_frame=config_params["frame_name"])

    @classmethod
    def _dict_meets_reqs(clazz, param_dict: dict):
        return set(clazz.param_reqs).issubset(set(param_dict.keys()))

    def __str__(self):
        return f"{self.name}\nTransform: {self.parent_frame} -> {self.frame_name}\n"
