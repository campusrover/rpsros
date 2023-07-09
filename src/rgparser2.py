#!/usr/bin/env python3
"""
Package of very simple parsing used in Robogym and friends
"""
from json import JSONDecodeError, loads
from prompt_toolkit import PromptSession

class Parser:

    """General very simple parser for commands to rgserver"""
    def __init__(self, initial_variables, command_table):
        # Initialize an empty dictionary to store variables and their values
        self.variables = initial_variables
        self.session = None
        self.command_info = None
        self.session = PromptSession()

        # Define the built-in command table
        self.local_command_table = {
            "help": {
                "args": [],
                "nargs": 0,
                "description": "Show available commands",
                "handler": self.print_commands,
            },
            "set": {
                "args": ["<variable>", "<value>"],
                "nargs": 2,
                "description": "Set a variable to a value",
                "handler": self.set_variable,
                "usage": "Invalid command. Usage: set <varname> <value>"
            },
            "show": {
                "nargs": 0,
                "args": ["[<variable>]"],
                "description": "Show all variables or the value of a specific variable",
                "handler": self.show_variable,
            },
            "reset": {
                "args": [],
                "nargs": 0,
                "description": "Reset the state",
                "handler": self.reset,
            },
        }
        self.command_table = { **self.local_command_table, **command_table}  

    def print_commands(self, _):
        """Display all the commands we know about"""
        print("\n".join([f"- {command} {' '.join(info['args'])}: {info['description']}" for command, info in self.command_table.items()]))


    def reset(self, args):
        """Reset the state of the commands"""
        self.variables.clear()
        print("State has been reset.")

    def set_variable(self, args):
        """Set a variable to a value. Legal values are words and numbers"""
        parse_args = self.get_tokens(args)
        if not parse_args:
            print(self.command_info["usage"])
            return False
        else:
            self.variables[parse_args[0]] = parse_args[1]
            print(f"Variable '{parse_args[0]}' has been set to '{parse_args[1]}'.")
            return True

    def show_variable(self, args):
        """Show the value of a variable"""
        parse_args = self.get_tokens(args)
        if len(parse_args) == 0 and len(self.variables) > 0:
            print("\n".join([f"{name}: {value}" for name, value in self.variables.items()]))
            return True
        if len(self.variables) == 0:
            print("No variables have been set.")
            return False
        if len(parse_args) == 1:
            variable_name = parse_args[0]
            if variable_name in self.variables:
                print(f"The value of '{variable_name}' is '{self.variables[variable_name]}'.")
                return True
            print(f"Variable '{variable_name}' has not been set.")
            return False
        print(self.command_info["usage"], "set_variable")
        return False

    def get_tokens(self, val_list):
        """parse a list of values, or return False if there's a parse error"""
        result = []
        for val in val_list:
            a_val = self.get_token(val)
            if a_val is False:
                return False
            else:
                result.append(a_val)
        return result

    def get_token(self, token):
        """Parse a value from a token. Either a word or a float or a list"""
        # Check if the token is a variable name, and if so, convert it to its digital value
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
        
    def cli(self):
        """Main entry point to parse commands"""
        while True:
            command, args = self.get_command()
            if command not in self.command_table:
                print("Invalid command. Type 'help' to see the available commands.")
                continue
            # Execute the command
            self.command_info = self.command_table[command]
            self.command_info["handler"](args)
