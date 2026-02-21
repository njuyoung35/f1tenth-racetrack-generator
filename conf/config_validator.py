import json
from pathlib import Path
from typing import Any, Dict

import jsonschema
import yaml


class ConfigValidator:
    """Validates YAML configuration against JSON schema"""

    def __init__(self, schema_path: str = None):
        if schema_path is None:
            schema_path = Path(__file__).parent / "config.schema.json"

        with open(schema_path, "r") as f:
            self.schema = json.load(f)

    def validate(self, config_path: str) -> Dict[str, Any]:
        """
        Validate a YAML config file against the schema

        Args:
            config_path: Path to YAML config file

        Returns:
            Validated configuration dictionary

        Raises:
            jsonschema.ValidationError: If config doesn't match schema
            yaml.YAMLError: If YAML is malformed
        """
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # Validate against schema
        jsonschema.validate(instance=config, schema=self.schema)

        return config

    def load_with_defaults(self, config_path: str) -> Dict[str, Any]:
        """
        Load config and fill in missing values with schema defaults

        Args:
            config_path: Path to YAML config file

        Returns:
            Configuration with defaults applied
        """
        # First validate the provided config
        config = self.validate(config_path)

        # Apply defaults from schema (you'd need to traverse the schema)
        # This is a simplified version - a full implementation would need
        # to recursively apply defaults
        config_with_defaults = self._apply_defaults(config, self.schema)

        return config_with_defaults

    def _apply_defaults(self, instance: Any, schema: Dict) -> Any:
        """Recursively apply defaults from schema to instance"""
        if isinstance(instance, dict) and "properties" in schema:
            result = {}
            for prop_name, prop_schema in schema["properties"].items():
                if prop_name in instance:
                    result[prop_name] = self._apply_defaults(
                        instance[prop_name], prop_schema
                    )
                elif "default" in prop_schema:
                    result[prop_name] = prop_schema["default"]
            return result
        elif isinstance(instance, list) and "items" in schema:
            return [self._apply_defaults(item, schema["items"]) for item in instance]
        else:
            return instance


def generate_example_config(schema_path: str = None) -> str:
    """
    Generate an example YAML config from the schema

    This is useful for documentation or creating starter configs
    """
    if schema_path is None:
        schema_path = Path(__file__).parent / "config.schema.json"

    with open(schema_path, "r") as f:
        schema = json.load(f)

    # This would need a recursive function to build example from schema
    # For now, return a minimal example
    return """
# Generated example configuration - see schema for all options
config:
  space:
    width: 20.0
    height: 10.0

  centerline:
    n_points: 60
    max_lloyd_iterations: 10
    curvature_threshold: 0.3

  walls:
    duct_thickness: 0.30
    min_track_width: 3.00
    width_model:
      components:
        - type: constant
          value_dist:
            type: uniform
            low: 1.0
            high: 2.0

  output:
    visualize: false
"""
