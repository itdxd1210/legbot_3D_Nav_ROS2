class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10
    # The bundled GPMP extension is not ABI-safe with the current Jazzy
    # dependency set. Keep the tomogram multi-layer A* route and let SCAN
    # provide the executable local B-spline and online collision avoidance.
    optimize_path = False
    route_simplify_epsilon = 0.08


class ConfigWrapper():
    tomo_dir = '/src/tomogram/'


class Config():
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()
