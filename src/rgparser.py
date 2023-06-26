#!/usr/bin/env python3
"""
Package of very simple parsing used in Robogym and friends
"""
from prompt_toolkit import PromptSession

class CommandInterface:
    """General very simple parser for commands to rgserver"""
    def __init__(self, initial_variables):
        # Initialize an empty dictionary to store variables and their values
        self.variables = initial_variables
        self.session = None
        self.command_info = None

        # Define the command table
        self.command_table = {
            "reset": {
                "args": [],
                "description": "Reset the state",
                "handler": self.reset,
            },
            "set": {
                "args": ["<variable>", "<value>"],
                "description": "Set a variable to a value",
                "handler": self.set_variable,
            },
            "show": {
                "args": ["[<variable>]"],
                "description": "Show all variables or the value of a specific variable",
                "handler": self.show_variable,
            },
            "move": {
                "args": ["<forward_speed>", "<distance>"],
                "description": "Move with a given forward speed and distance",
                "handler": self.move,
            },
            "help": {
                "args": [],
                "description": "Show available commands",
                "handler": self.print_commands,
            },
            "quit": {
                "args": [],
                "description": "Quit the program",
                "handler": self.quit,
            },
            "stop": {
                "args": [],
                "description": "Stop the robot",
                "handler": self.stop,
            },
            "goto": {
                "args": ["<x>", "<y>"],
                "description": "Go to given odometry coordinate",
                "handler": self.goto,
                "usage": "Invalid command. Usage: goto <x> <y>"
            },


        }

    def get_value(self, token):
        """Parse a value from a token. Either a word or a float"""
        # Check if the token is a variable name
        if token in self.variables:
            return float(self.variables[token])
        # If not, assume it's a float value
        try:
            return float(token)
        except ValueError:
            return None

    def invalid_command(self, command):
        pass

    def reset(self):
        self.variables.clear()
        return True, "State has been reset.", "reset", {}

    def set_variable(self, args):
        if len(args) == 2:
            variable_name = args[0]
            variable_value = self.get_value(args[1])
            if variable_value is not None:
                self.variables[variable_name] = variable_value
                return True, f"Variable '{variable_name}' has been set to '{variable_value}'.", "set", {}
            else:
                return False, "Invalid value. Please provide a valid float or existing variable.", "set", {}
        else:
            return False, "Invalid command. Usage: set <variable> <value>", "set", {}

    def show_variable(self, args):
        if len(args) == 0:
            if self.variables:
                variables_info = "\n".join([f"{name}: {value}" for name, value in self.variables.items()])
                return True, f"Variables:\n{variables_info}", "show", {}
            else:
                return True, "No variables have been set.", "show", {}
        elif len(args) == 1:
            variable_name = args[0]
            if variable_name in self.variables:
                return True, f"The value of '{variable_name}' is '{self.variables[variable_name]}'.", "show", {}
            else:
                return False, f"Variable '{variable_name}' has not been set.", "show", {}
        else:
            return False, "Invalid command. Usage: show or show <variable>", "show", {}

    def move(self, args):
        """Parse the move command"""
        if len(args) == 2:
            forward_speed = self.get_value(args[0])
            distance = self.get_value(args[1])
            if forward_speed is not None and distance is not None:
                params = {
                    "forward_speed": forward_speed,
                    "distance": distance,
                }
                return True, f"Moving with forward speed '{forward_speed}' for distance '{distance}'.", "move", params
                # Implement the logic for moving here
            else:
                return False, "Invalid value. Please provide valid floats or existing variables.", "move", {}
        else:
            return False, "Invalid command. Usage: move <forward_speed> <distance>", "move", {}

    def print_commands(self, _):
        """Display all the commands we know about"""

        commands_info = "\n".join([f"- {command} {' '.join(info['args'])}: {info['description']}" for command, info in self.command_table.items()])
        return True, f"Available commands:\n{commands_info}", "help", {}

    def quit(self, _):
        return True, "Quitting the program...", "quit", {}

    def stop(self, _):
        return True, "Stop the robot", "stop", {}

    def goto(self, args):
        if len(args) == 2:
            x = self.get_value(args[0])
            y = self.get_value(args[1])
            if x is not None and y is not None:
                params = {"x": x, "y": y}
                return True, f"Going to requested coordinates x: {x:2.2f}, y: {y:2.2f}", "goto", params
            else:
                return False, "Invalid values for x or y. Please correct", "goto", {}
        else:
            return False, "Invalid command. Usage: goto <x> <y>", "goto", {}


    def get_command(self):
        while True:
            try:
                raw_input = self.session.prompt(">>> ")
                command_input = raw_input.strip().split()
                if len(command_input) > 0:
                    command = command_input[0]
                    args = command_input[1:]
                    return command, args
                else:
                    return None, None
            except KeyboardInterrupt:
                continue
        
    def command(self):
        self.session = PromptSession()

        while True:
            command, args = self.get_command()
            if command not in self.command_table:
                print("Invalid command. Type 'help' to see the available commands.")
                continue

            # Execute the command
            self.command_info = self.command_table[command]
            status, message, result, params = self.command_info["handler"](args)
            values = {**params, **self.variables}
            if status:
                print(message)
                if command in ("move", "stop", "goto", "quit", "exit"):
                    return command, message, result, values
            else:
                print("Error:", message)
