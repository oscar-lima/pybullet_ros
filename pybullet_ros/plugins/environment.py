#!/usr/bin/env python3

"""Plugin responsible for loading the simulation environment."""

from typing import Any, Dict


class Environment:
    def __init__(self, node, pybullet, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.config: Dict[str, Any] = kwargs.get('config', {})
        self.use_deformable_world = bool(kwargs.get('use_deformable_world', False))

        if self.use_deformable_world:
            self.logger.info('Using deformable world (soft body simulation)')
            self.pb.resetSimulation(self.pb.RESET_USE_DEFORMABLE_WORLD)

    def load_environment(self) -> None:
        gravity = float(self.config.get('gravity', -9.81))
        self.pb.setGravity(0, 0, gravity)
        self.pb.loadURDF('plane.urdf')
        self.load_environment_via_code()

    def load_environment_via_code(self) -> None:
        """Override in derived classes to spawn custom objects."""
        # Intentionally left blank to be extended by user plugins.
        return
