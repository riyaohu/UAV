"""
Main entry point for UAV Search Simulator
"""
from .simulator import Simulator
from . import config
from .utils.seed import set_seed

def main():
    """Main function"""
    set_seed(config.DEFAULT_SEED)
    # Create and run simulator
    simulator = Simulator(render=True, mode="demo", use_grid_map=False ,algorithm_name="template_matching")
    simulator.run()


if __name__ == "__main__":
    main()
