class Params:
    HEADLESS = False

    RUN_TIME = 25  # Seconds
    FRAME_RATE = 60

    SURFACE_DIMS_M = (50, 25)
    PX_PER_M = 30

    WINDOW_DIMS = (SURFACE_DIMS_M[0] * PX_PER_M,
                   SURFACE_DIMS_M[1] * PX_PER_M)
    pass


PARAMS = Params()
