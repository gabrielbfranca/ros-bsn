from pytest_bdd import scenarios
import os

# Load the feature file
scenarios(os.path.join(os.path.dirname(__file__), 'features/hub_node_integration.feature'))
