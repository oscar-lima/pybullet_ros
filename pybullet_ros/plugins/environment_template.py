from pybullet_ros.plugins.environment import Environment as DefaultEnv


class Environment(DefaultEnv):
    def __init__(self, node, pybullet, **kwargs):
        super().__init__(node, pybullet, **kwargs)

    def load_environment_via_code(self):
        self.logger.warn('Loading custom environment via code!')
        # Example:
        # self.pb.loadURDF(...)
        # self.pb.setTimeStep(0.001)
