from prompt_toolkit import PromptSession
from math import exp
def main():
    exp(1)
    session = PromptSession()
    commands = []
    while True:
        try:
            cmd = session.prompt(">>> ")
            if cmd == "exit":
                break
            commands.append(cmd)
            print(f"Executing command: {cmd}")
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt")
            break

if __name__ == "__main__":
    main()