#!/usr/bin/env python3

import json
import websocket
import logging
import pprint

# create panda_client logger.
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s')
logger = logging.getLogger('panda_client')

class PandaClient:
    def __init__(self, ros_box_address='127.0.0.1', port=9090):
        # start a websocket connection with the ROS machine.
        self.conn = websocket.create_connection(
            "ws://"+ros_box_address+":"+str(port))
        # create dictionary for storing tasks.
        self.tasks_dict = {}

    # load tasks from a json file.
    def load_tasks(self, tasks_json_file):
        try:
            with open(tasks_json_file, 'r') as tasks_json:
                self.tasks_dict.update(json.load(tasks_json))
        except IOError:
            logger.exception("Error opening file '{}'!".format(tasks_json_file))
            return
        logger.info("Loaded tasks from '{}'".format(tasks_json_file))

    # add a task to the task dictioary.
    def add_task(self, task_name, task_list):
        logger.info("Adding task '{}' to dictionary".format(task_name))
        self.tasks_dict[task_name] = task_list

    # delete a task from the task dictionary.
    def remove_task(self, task_name):
        try:
            del self.tasks_dict[task_name]
        except KeyError:
            logger.exception("task name '{}' is not present in the dictionary!".format(task_name))
            return
        logger.info("Removed task '{}'".format(task_name))

    # save the task dictionary to a json file.
    def save_tasks(self, tasks_json_file):
        try:
            with open(tasks_json_file, 'w') as tasks_json:
                json.dump(self.tasks_dict, tasks_json)
        except IOError:
            logger.exception("Error writing to file '{}'!".format(tasks_json_file))
            return
        logger.info("Saved tasks to '{}'".format(tasks_json_file))

    # call services on remote ubuntu machine
    def execute_task(self, task_name):
        try:
            task = self.tasks_dict[task_name]
        except KeyError:
            logger.exception("task name '{}' is not present in the dictionary!").format(task_name)
            return
        if not isinstance(task, list):
            logger.error("Task command '{}' must be a list of service calls!").format(task_name)
            return
        for srv_req in task:
            service_name = srv_req.pop("service")
            call_service_msg = {
                "op": "call_service",
                "service": service_name,
                "args": srv_req
            }
            json_msg = json.dumps(call_service_msg)
            logger.info("[{}] calling service '{}'".format(task_name, service_name))
            self.conn.send(json_msg)
            service_response = self.conn.recv()
            service_response = json.loads(service_response)
            logger.info(service_response)
        return service_response

        # list all tasks currently in the database.
    def list_tasks(self):
        if len(self.tasks_dict) is 0:
            logger.info("No tasks were found. Can not list tasks.")
        else:
            logger.info("Tasks: ")
            for task in self.tasks_dict:
                print(task)

    def print_task(self, task_name):
        task = self.tasks_dict[task_name]
        if task is None:
            logger.error("Task: '{}' was not found in the database. Can not print task.".format(task_name))
        else:
            pprint.pprint(task)