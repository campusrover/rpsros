#!/usr/bin/env python3
"""
Package of very simple parsing used in Robogym and friends
"""
from json import JSONDecodeError, loads
from prompt_toolkit import PromptSession

class Parser:
    """General very simple parser for commands to rgserver"""
    def __init__(self, initial_variables):
        # Initialize an empty dictionary to store variables and their values
        self.variables = initial_variables
        self.session = None
        self.command_info = None
        self.session = PromptSession()

        # Define the command table
        self.command_table = {
            "route": {
                "args": ["<route>"],
                "description": "Specific list of points. List can be zero or more points",
                "handler": self.route,
                "usage": "Invalid command. Usage: route <array of points>"
            },
            "goto": {
                "args": ["<x>", "<y>"],
                "description": "Go to given odometry coordinate",
                "handler": self.goto,
                "usage": "Invalid command. Usage: goto <x> <y>"
            },
            "move": {
                "args": ["<forward_speed>", "<distance>"],
                "description": "Move with a given forward speed and distance",
                "handler": self.move,
                "usage": "Invalid command. Usage: move <speed> <distance>"
            },
            "stop": {
                "args": [],
                "description": "Stop the robot",
                "handler": self.stop,
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
            "set": {
                "args": ["<variable>", "<value>"],
                "description": "Set a variable to a value",
                "handler": self.set_variable,
                "usage": "Invalid command. Usage: set <varname> <value>"
            },
            "show": {
                "args": ["[<variable>]"],
                "description": "Show all variables or the value of a specific variable",
                "handler": self.show_variable,
            },
            "reset": {
                "args": [],
                "description": "Reset the state",
                "handler": self.reset,
            },
        }

    def invalid_command(self, command):
        """Simply called when there's an invalid command"""
        pass

    def reset(self):
        """Reset the state of the commands"""
        self.variables.clear()
        return True, "State has been reset.", "reset", {}

    def set_variable(self, args):
        """Set a variable to a value. Legal values are words and numbers"""
        parse_args = self.get_tokens(args)
        if not parse_args:
            return False, self.command_info["usage"], "move", {}
        else:
            self.variables[parse_args[0]] = parse_args[1]
            return True, f"Variable '{parse_args[0]}' has been set to '{parse_args[1]}'.", "set", {}

    def show_variable(self, args):
        """Show the value of a variable"""
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
        parse_args = self.get_tokens(args)
        if (not parse_args):
            return False, self.command_info["usage"], "move", {}
        else:
            return True, f"Moving with forward speed: {parse_args[0]:2.2f}, distance: {parse_args[1]:2.2f}", "move", {"forward_speed": parse_args[0], "distance": parse_args[1]}

    def print_commands(self, _):
        """Display all the commands we know about"""
        commands_info = "\n".join([f"- {command} {' '.join(info['args'])}: {info['description']}" for command, info in self.command_table.items()])
        return True, f"Available commands:\n{commands_info}", "help", {}

    def quit(self, _):
        """Parse the Quit command"""
        return True, "Quitting the program...", "quit", {}

    def stop(self, _):
        """Parse the Stop command"""
        return True, "Stop the robot", "stop", {}

    def goto(self, args):
        """Command parser to go to specific coordinate"""
        parse_args = self.get_tokens(args)
        if (not parse_args):
            return False, self.command_info["usage"], "route", {}
        else:
            return True, f"Going to requested coordinates x: {parse_args[0]:2.2f}, y: {parse_args[1]:2.2f}", "goto", {"x": parse_args[0], "y": parse_args[1]}

    def route(self, args):
        """Parse the route command"""
        parse_args = self.get_tokens(args)
        if (not parse_args):
            return False, self.command_info["usage"], "route", {}
        else:
            return True, f"Executing Route: {str(parse_args[0])}", "route", {"args": parse_args}

    def get_tokens(self, val_list):
        """parse a list of values, or return False if there's a parse error"""
        result = []
        for val in val_list:
            a_val = self.get_token(val)
            print(a_val)
            if a_val is False:
                return False
            else:
                result.append(a_val)
        return result

    def get_token(self, token):
        """Parse a value from a token. Either a word or a float or a list"""
        # Check if the token is a variable name, and if so, convert it to its digital value
        print("get token", token)
        if token.isidentifier():
            return token
        try:
            return float(token)
        except ValueError:
            pass
        # if not, see if works as a json value
        try:
            return (loads(token))
        except JSONDecodeError:
            print(f"bad json: {token}")
            return False
        return False

    def get_command(self):
        """Prompt for the next command and then parse the command"""
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
        """Main entry point to parse commands"""
        while True:
            command, args = self.get_command()
            if command not in self.command_table:
                print("Invalid command. Type 'help' to see the available commands.")
                continue

            # Execute the command
            self.command_info = self.command_table[command]
            status, message, result, params = self.command_info["handler"](args)
            print(params, self.variables)
            values = {**params, **self.variables}
            if status:
                print(message)
                if command in ("move", "stop", "goto", "quit", "exit"):
                    return command, message, result, values
            else:
                print("Error:", message)
