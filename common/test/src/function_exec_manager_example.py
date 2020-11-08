#!/usr/bin/env python3

"""
Example of how to use function_exec_manager.py
"""

import time
from pybullet_ros.function_exec_manager import FuncExecManager

class Plugin1:
    def __init__(self, pybullet, robot, **kargs):
        print('executing plugin 1 constructor')

    def execute(self):
        print('calling plugin1 execute function')
        time.sleep(1)

class Plugin2:
    def __init__(self, pybullet, robot, **kargs):
        print('executing plugin 2 constructor')

    def execute(self):
        print('calling plugin2 execute function')
        time.sleep(3)

class Plugin3:
    def __init__(self, pybullet, robot, **kargs):
        print('executing plugin 3 constructor')

    def execute(self):
        print('calling plugin3 execute function')
        time.sleep(7)

class TestClass:
    def __init__(self):
        self.i = 0

    def count_to_five(self):
        self.i = self.i + 1
        if self.i > 5:
            return True
        else:
            return False

    def exec_after_each_loop(self):
        pass

    def pause_execution(self):
        return False

if __name__ == '__main__':
    list_of_classes = [Plugin1, Plugin2, Plugin3]
    # make list of objects
    list_of_objects = []
    for c in list_of_classes:
        list_of_objects.append(c('pb', 'robot'))
    # create stop condition
    hc = TestClass()
    obj = FuncExecManager(list_of_objects, hc.count_to_five, hc.exec_after_each_loop, hc.pause_execution)
    # start parallel execution of all "execute" class methods in a synchronous way
    obj.start_synchronous_execution(loop_rate=0.25)
    print('bye, bye!')
