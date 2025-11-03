class Params:
    HEADLESS = False

    RUN_TIME = 15
    FRAME_RATE = 60

    SURFACE_DIMS_M = (50, 50)
    PX_PER_M = 30

    PADDING_PX = 25
    WINDOW_DIMS = (min(SURFACE_DIMS_M[0] * PX_PER_M + 2 * PADDING_PX, 1920),
                   min(SURFACE_DIMS_M[1] * PX_PER_M + 2 * PADDING_PX, 1080))
    pass


PARAMS = Params()
