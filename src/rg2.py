class CommandInterface:
    def __init__(self):
        # Initialize an empty dictionary to store variables and their values
        self.variables = {}

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
        }

    def get_command(self):
        while True:
            # Prompt the user for a command
            command = input(">> ")

            # Split the command into tokens
            tokens = command.split()

            # Check if the command exists in the command table
            if tokens[0] in self.command_table:
                command_info = self.command_table[tokens[0]]
                expected_args = command_info["args"]

                if len(tokens[1:]) != len(expected_args):
                    print(f"Invalid command. Usage: {tokens[0]} {' '.join(expected_args)}")
                else:
                    return (tokens[0], tokens[1:])
            else:
                print("Invalid command. Please try again.")

    # Rest of the command handler methods...

# Create an instance of the CommandInterface class
command_interface = CommandInterface()

while True:
    command, args = command_interface.get_command()
    # Execute the command
    command_info = command_interface.command_table[command]
    status, result, params = command_info["handler"](args)
    if status:
        print(result)
        if params:
            print("Parameters:", params)
    else:
        print("Error:", result)
