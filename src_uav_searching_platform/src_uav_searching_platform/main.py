"""
Main entry point for UAV Search Simulator
"""
from .simulator import Simulator



def main():
    """Main function"""
    # Create and run simulator
    simulator = Simulator(render=True, mode="demo")
    simulator.run()


if __name__ == "__main__":
    main()
