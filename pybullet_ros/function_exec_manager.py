#!/usr/bin/env python3

"""
Imagine you have to run multiple functions within a deadline.
Running them sequentially will be the easiest thing to do, but if one of them behaves badly
and takes too much time, we are in trouble...

This code, instead of runing functions sequentially, we run them all on separate threads in parallel.
This allows us to consistently call (in time) the functions that meet their deadline, while
ignoring those functions that miss the cycle rate, yet still call them at their maximum rate.

Example:

             plugin1       plugin2         plugin3
start    ----------------------------------------------
              start        start           start
                
              finish
                
                                           finish




loop rate ----------------------------------------------  (deadline)
              start again  start again
                                           finish
                                           start again

                                        .
                                        .
                                        . etc...

  In this example plugin1 and 2 meet their deadline (and therefore are "in time"),
  while plugin3 takes more time than the desired call frequency (misses the loop rate), yet
  it is still being called as fast as it can make its computation.

See function_exec_manager_example.py for a runing example on how to use this class
"""

from threading import Thread
import time, sys, copy

class FuncExecManager:
    """
    helper class to keep track of synchronous multiple parallel execution of functions with deadlines
    """
    def __init__(self, list_of_objects, stop_condition, exec_after_each_loop, pause_execution, log_info=print,
                 log_warn=print, log_debug=print, function_name='function'):
        # deadline for functions to finish their process
        self.loop_rate = 0.25 # every 4 secs
        # keep track of functions which execution time is "below" the deadline (set by loop rate)
        self.on_time_functions = []
        # keep track of functions which execution time is "above" the deadline (set by loop rate)
        self.late_functions = []
        self.late_threads = []
        # flag to know when a cycle is finished
        self.is_loop_finished = False
        # to differentiate between each cycle (and keep track of functions that finish on time)
        self.cycle_unique_id = None
        # save in member variable the received list of objects
        self.list_of_objects = list_of_objects
        # the function that will be called to check if algorithm should stop or continue
        self.stop_condition = stop_condition
        # this function will be called after each loop
        self.exec_after_each_loop = exec_after_each_loop
        # offer the user the possibility to pause the execution
        self.pause_execution = pause_execution
        # configure loggers
        log_info(f'Started synchronous {function_name} execution manager')
        self.log_warn = log_warn
        self.log_debug = log_debug

    def time_control(self, cycle_unique_id, obj):
        class_name = str(obj.__class__)
        # make backup of wall time
        start_time = time.time()
        obj.execute()
        # compare id to check if we are still on time
        if self.cycle_unique_id == cycle_unique_id:
            self.log_debug(f'finished {class_name} in time')
            self.on_time_functions.append(obj)
        else:
            end_time = time.time()
            self.log_warn(f'{class_name}: Missed loop rate, took {str(round(end_time - start_time - (1.0 / self.loop_rate), 2))} sec longer than expected')
            self.late_functions.append(obj)

    def loop_thread(self):
        """
        we create an additional thread to monitor the deadline
        """
        # convert frequency to time
        time.sleep(1.0 / self.loop_rate)
        self.log_debug('==== loop rate! ====')
        # raise flag to indicate that one loop is complete
        self.is_loop_finished = True

    def start_synchronous_execution(self, loop_rate=0.25):
        # allow user to modify the frequency at which the functions will be called
        self.loop_rate = loop_rate
        # at first all functions are on time
        self.on_time_functions = self.list_of_objects
        self.late_functions = []
        # init cycle_unique_id
        cycle_unique_id = 0
        if cycle_unique_id > sys.maxsize:
            cycle_unique_id = 0
        # run x iterations until user wants to stop (you might want to pass here a ctrl + c detection)
        while not self.stop_condition():
            # initialize flag
            self.is_loop_finished = False
            # jump to the next cycle id (because previous loop is finished)
            cycle_unique_id = cycle_unique_id + 1
            self.cycle_unique_id = cycle_unique_id
            # start "on time" functions
            thread_list = []
            for func in self.on_time_functions:
                thread_list.append(Thread(target=self.time_control, args=(cycle_unique_id, func,)))
            self.on_time_functions = []
            for t in thread_list:
                t.start()
            # run loop function on a separate thread
            Thread(target=self.loop_thread).start()
            # wait until loop finishes
            while not self.is_loop_finished:
                # if there is a late thread that finished, run it again right away
                if self.late_functions != []:
                    self.late_threads = []
                    for func in self.late_functions:
                        self.late_threads.append(Thread(target=self.time_control, args=(cycle_unique_id, func,)))
                        self.late_threads[-1].start()
                    self.late_functions = []
                # sleep to reduce computational load
                time.sleep(0.001)
                # check if user wants to pause execution
                while self.pause_execution():
                    time.sleep(0.1)
            # execute custom function after having finished the loop
            self.exec_after_each_loop()
        # wait for threads to finish
        for thread in self.late_threads:
            thread.join()
