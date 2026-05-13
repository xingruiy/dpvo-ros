import yaml


class Config:
    def merge_from_file(self, path):
        with open(path, 'r', encoding='utf-8') as config_file:
            values = yaml.safe_load(config_file) or {}

        for key, value in values.items():
            setattr(self, key, value)

    def merge_from_list(self, values):
        if len(values) % 2 != 0:
            raise ValueError("Configuration overrides must be key/value pairs")

        for key, value in zip(values[0::2], values[1::2]):
            setattr(self, key, yaml.safe_load(value))


_C = Config()

# max number of keyframes
_C.BUFFER_SIZE = 4096

# bias patch selection towards high gradient regions?
_C.CENTROID_SEL_STRAT = 'RANDOM'

# VO config (increase for better accuracy)
_C.PATCHES_PER_FRAME = 80
_C.REMOVAL_WINDOW = 20
_C.OPTIMIZATION_WINDOW = 12
_C.PATCH_LIFETIME = 12

# threshold for keyframe removal
_C.KEYFRAME_INDEX = 4
_C.KEYFRAME_THRESH = 12.5

# camera motion model
_C.MOTION_MODEL = 'DAMPED_LINEAR'
_C.MOTION_DAMPING = 0.5

_C.MIXED_PRECISION = True

# Loop closure
_C.LOOP_CLOSURE = False
_C.BACKEND_THRESH = 64.0
_C.MAX_EDGE_AGE = 1000
_C.GLOBAL_OPT_FREQ = 15

# Classic loop closure
_C.CLASSIC_LOOP_CLOSURE = False
_C.LOOP_CLOSE_WINDOW_SIZE = 3
_C.LOOP_RETR_THRESH = 0.04

cfg = _C
