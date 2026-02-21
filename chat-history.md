# config.schema.json 작성

## prompt

for writing config.yaml, i want unity of the information. user see README.md and follow. they expect that might be absolute true. but it is not 'programmized' truth. author and maintainer maintain these. but it can be automated, by config.yaml schema. so lets edit README.md's input parameter section, just leave link to config.yaml schema. python code should also read that schema. schema also has default values and its own documentation with commentary. let's write the file. follow the standard schema file for yaml

## response

````markdown
Excellent approach! You're absolutely right - having a single source of truth through a schema file is much more maintainable and reliable. Let's create a JSON Schema for the config.yaml that will serve as both documentation and validation.

## 1. First, let's create the schema file

Create `conf/config.schema.json`:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://github.com/yourusername/f1tenth-racetrack-generator/conf/config.schema.json",
  "title": "F1Tenth Racetrack Generator Configuration",
  "description": "Configuration schema for the F1Tenth racetrack generator tool",
  "type": "object",
  "required": ["config"],
  "additionalProperties": false,

  "definitions": {
    "distribution": {
      "oneOf": [
        {
          "type": "object",
          "properties": {
            "type": { "enum": ["uniform"] },
            "low": { "type": "number", "description": "Lower bound of uniform distribution" },
            "high": { "type": "number", "description": "Upper bound of uniform distribution" }
          },
          "required": ["type", "low", "high"],
          "additionalProperties": false
        },
        {
          "type": "object",
          "properties": {
            "type": { "enum": ["normal"] },
            "mean": { "type": "number", "description": "Mean of normal distribution" },
            "std": { "type": "number", "description": "Standard deviation of normal distribution" }
          },
          "required": ["type", "mean", "std"],
          "additionalProperties": false
        },
        {
          "type": "object",
          "properties": {
            "type": { "enum": ["halfnormal"] },
            "sigma": { "type": "number", "description": "Scale parameter of half-normal distribution" }
          },
          "required": ["type", "sigma"],
          "additionalProperties": false
        },
        {
          "type": "object",
          "properties": {
            "type": { "enum": ["fixed"] },
            "value": { "type": "number", "description": "Fixed value" }
          },
          "required": ["type", "value"],
          "additionalProperties": false
        }
      ]
    },

    "generator_component": {
      "type": "object",
      "properties": {
        "name": { "type": "string", "description": "Component name for debugging" },
        "type": {
          "type": "string",
          "enum": ["constant", "sinusoidal", "curvature_scaled", "noise", "random_spline"],
          "description": "Type of width generator component"
        }
      },
      "required": ["type"],
      "allOf": [
        {
          "if": {
            "properties": { "type": { "const": "constant" } }
          },
          "then": {
            "properties": {
              "value_dist": { "$ref": "#/definitions/distribution" }
            },
            "required": ["value_dist"]
          }
        },
        {
          "if": {
            "properties": { "type": { "const": "sinusoidal" } }
          },
          "then": {
            "properties": {
              "amplitude_dist": { "$ref": "#/definitions/distribution" },
              "frequency_dist": { "$ref": "#/definitions/distribution" },
              "phase_dist": {
                "$ref": "#/definitions/distribution",
                "description": "Phase in radians"
              }
            },
            "required": ["amplitude_dist", "frequency_dist", "phase_dist"]
          }
        },
        {
          "if": {
            "properties": { "type": { "const": "curvature_scaled" } }
          },
          "then": {
            "properties": {
              "sensitivity_dist": { "$ref": "#/definitions/distribution" },
              "base_width_dist": { "$ref": "#/definitions/distribution" }
            },
            "required": ["sensitivity_dist"]
          }
        },
        {
          "if": {
            "properties": { "type": { "const": "noise" } }
          },
          "then": {
            "properties": {
              "std_dev_dist": { "$ref": "#/definitions/distribution" },
              "correlation_length": {
                "type": "number",
                "minimum": 0,
                "maximum": 1,
                "default": 0.2,
                "description": "Correlation length in normalized t space [0,1]"
              }
            },
            "required": ["std_dev_dist"]
          }
        },
        {
          "if": {
            "properties": { "type": { "const": "random_spline" } }
          },
          "then": {
            "properties": {
              "n_control_points": {
                "type": "integer",
                "minimum": 3,
                "maximum": 20,
                "default": 8,
                "description": "Number of control points for spline"
              },
              "control_point_range": {
                "type": "object",
                "properties": {
                  "min": { "type": "number", "default": 0.3 },
                  "max": { "type": "number", "default": 2.0 }
                },
                "required": ["min", "max"]
              },
              "smoothness": {
                "type": "number",
                "minimum": 0,
                "maximum": 1,
                "default": 0.5,
                "description": "Spline smoothing factor"
              }
            },
            "required": ["n_control_points", "control_point_range"]
          }
        }
      ],
      "additionalProperties": false
    }
  },

  "properties": {
    "config": {
      "type": "object",
      "description": "Main configuration object",
      "additionalProperties": false,

      "properties": {
        "space": {
          "type": "object",
          "description": "Physical space parameters",
          "additionalProperties": false,
          "properties": {
            "width": {
              "type": "number",
              "minimum": 5,
              "maximum": 100,
              "default": 20.0,
              "description": "Width of the rectangular space (meters)"
            },
            "height": {
              "type": "number",
              "minimum": 5,
              "maximum": 100,
              "default": 10.0,
              "description": "Height of the rectangular space (meters)"
            }
          },
          "required": ["width", "height"]
        },

        "centerline": {
          "type": "object",
          "description": "Centerline generation parameters",
          "additionalProperties": false,
          "properties": {
            "n_points": {
              "type": "integer",
              "minimum": 20,
              "maximum": 500,
              "default": 60,
              "description": "Number of initial random points for Voronoi generation"
            },
            "max_lloyd_iterations": {
              "type": "integer",
              "minimum": 0,
              "maximum": 100,
              "default": 10,
              "description": "Number of Lloyd's algorithm iterations"
            },
            "max_verifying_iterations": {
              "type": "integer",
              "minimum": 1,
              "maximum": 1000,
              "default": 50,
              "description": "Maximum attempts to generate valid centerline"
            },
            "virtual_grid_width": {
              "type": "number",
              "minimum": 1,
              "maximum": 20,
              "default": 5.0,
              "description": "Cell size for virtual grid region selection (meters)"
            },
            "virtual_grid_coverage": {
              "type": "number",
              "minimum": 0.1,
              "maximum": 1.0,
              "default": 0.8,
              "description": "Fraction of grid cells to sample from"
            },
            "curvature_threshold": {
              "type": "number",
              "minimum": 0.05,
              "maximum": 1.0,
              "default": 0.3,
              "description": "Maximum allowed curvature (1/meters)"
            }
          },
          "required": ["n_points", "max_lloyd_iterations", "curvature_threshold"]
        },

        "walls": {
          "type": "object",
          "description": "Wall (duct) generation parameters",
          "additionalProperties": false,
          "properties": {
            "duct_thickness": {
              "type": "number",
              "minimum": 0.05,
              "maximum": 0.5,
              "default": 0.30,
              "description": "Physical thickness of each duct (meters)"
            },
            "min_duct_length": {
              "type": "number",
              "minimum": 0.2,
              "maximum": 2.0,
              "default": 0.50,
              "description": "Minimum length of a duct segment (meters)"
            },
            "max_duct_length": {
              "type": "number",
              "minimum": 0.5,
              "maximum": 5.0,
              "default": 2.00,
              "description": "Maximum length of a duct segment (meters)"
            },
            "min_track_width": {
              "type": "number",
              "minimum": 1.0,
              "maximum": 10.0,
              "default": 3.00,
              "description": "Minimum allowed track width (meters)"
            },
            "wall_points_per_meter": {
              "type": "integer",
              "minimum": 10,
              "maximum": 200,
              "default": 50,
              "description": "Density of generated wall points (points per meter along wall)"
            },
            "connection_tolerance": {
              "type": "number",
              "minimum": 0.01,
              "maximum": 0.5,
              "default": 0.15,
              "description": "Tolerance for connecting duct segments (meters)"
            },
            "resolution": {
              "type": "number",
              "minimum": 0.01,
              "maximum": 0.2,
              "default": 0.05,
              "description": "Occupancy grid cell size (meters)"
            },
            "left_right_correlation": {
              "type": "number",
              "minimum": -1,
              "maximum": 1,
              "default": 0.8,
              "description": "Correlation between left and right wall widths (-1 to 1)"
            },
            "width_model": {
              "type": "object",
              "description": "Generative model for track width profile",
              "additionalProperties": false,
              "properties": {
                "components": {
                  "type": "array",
                  "description": "List of generator components to combine",
                  "items": { "$ref": "#/definitions/generator_component" },
                  "minItems": 1
                },
                "clipping": {
                  "type": "string",
                  "enum": ["max", "min", "none"],
                  "default": "max",
                  "description": "How to handle exceeding maximum width: clip to max, use min, or allow (unsafe)"
                }
              },
              "required": ["components"]
            }
          },
          "required": ["duct_thickness", "min_track_width", "width_model"]
        },

        "output": {
          "type": "object",
          "description": "Output and debugging parameters",
          "additionalProperties": false,
          "properties": {
            "output_dir": {
              "type": "string",
              "default": "output",
              "description": "Directory for saving output files"
            },
            "visualize": {
              "type": "boolean",
              "default": false,
              "description": "Enable debug visualizations"
            },
            "debug_output_dir": {
              "type": "string",
              "default": "debug",
              "description": "Directory for debug images"
            },
            "random_seed": {
              "type": ["integer", "null"],
              "minimum": 0,
              "maximum": 4294967295,
              "default": null,
              "description": "Random seed for reproducibility (null = random)"
            }
          },
          "required": ["visualize"]
        }
      },
      "required": ["space", "centerline", "walls", "output"]
    }
  }
}
```

## 2. Update README.md - Simplify Input Parameters Section

Replace the current Input Parameters section with:

```markdown
## 3. Input Parameters

All configuration is done via a YAML file (default: `conf/config.yaml`).
The configuration schema is **the single source of truth** for all parameters, their defaults, and documentation.

### Configuration Schema

The complete schema with detailed descriptions, default values, and constraints is maintained in:
[`conf/config.schema.json`](conf/config.schema.json)

This JSON Schema file:
- 📋 **Documents** every parameter with descriptions
- 🎯 **Defines** default values
- 🔒 **Specifies** constraints (min/max, allowed values)
- 🔗 **Serves** as the validation source for the Python code

### Example Configuration

A minimal working configuration looks like:

```yaml
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
        - type: sinusoidal
          amplitude_dist:
            type: halfnormal
            sigma: 0.3
          frequency_dist:
            type: uniform
            low: 1
            high: 5
          phase_dist:
            type: uniform
            low: 0
            high: 6.283

  output:
    visualize: true
    output_dir: "output"
    debug_output_dir: "debug"
```

For all available parameters, their types, and constraints, please refer to the **[JSON Schema file](conf/config.schema.json)** directly. This ensures you always have the most up-to-date and accurate information.
```

## 3. Create a validation utility

Create `conf/config_validator.py`:

```python
import json
import jsonschema
import yaml
from pathlib import Path
from typing import Dict, Any

class ConfigValidator:
    """Validates YAML configuration against JSON schema"""

    def __init__(self, schema_path: str = None):
        if schema_path is None:
            schema_path = Path(__file__).parent / "config.schema.json"

        with open(schema_path, 'r') as f:
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
        with open(config_path, 'r') as f:
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
                        instance[prop_name],
                        prop_schema
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

    with open(schema_path, 'r') as f:
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
```

## 4. Update main.py to use validation

```python
import sys
from pathlib import Path

# Add the project root to Python path
sys.path.append(str(Path(__file__).parent))

import numpy as np
from conf.config_validator import ConfigValidator
from centerline import CenterlineGenerator
from walls import WallGenerator

CONFIG_FILE_PATH = "./conf/config.yaml"

def main():
    # Validate and load configuration
    print("=" * 60)
    print("F1Tenth Track Generator")
    print("=" * 60)

    print("\n[Config] Validating configuration...")
    validator = ConfigValidator()

    try:
        config = validator.validate(CONFIG_FILE_PATH)
        config = config["config"]  # Extract the nested config
        print("  ✓ Configuration is valid")
    except Exception as e:
        print(f"  ✗ Configuration error: {e}")
        print("\nPlease check your config.yaml against the schema at conf/config.schema.json")
        return

    # Add derived/calculated parameters
    config["debug_output_dir"] = config["output"]["debug_output_dir"]
    config["visualize"] = config["output"]["visualize"]

    # Set random seed for reproducibility
    if config["output"].get("random_seed") is not None:
        np.random.seed(config["output"]["random_seed"])
        print(f"  ✓ Random seed set to: {config['output']['random_seed']}")

    # Print configuration summary
    print("\n[Config] Parameters:")
    print(f"  - Space: {config['space']['width']}m x {config['space']['height']}m")
    print(f"  - Centerline points: {config['centerline']['n_points']}")
    print(f"  - Duct thickness: {config['walls']['duct_thickness']}m")
    print(f"  - Min track width: {config['walls']['min_track_width']}m")

    # 1. 센터라인 생성
    print("\n[Phase 1] Generating centerline...")
    cg = CenterlineGenerator(config)
    centerline = cg.generate()

    # Create output directory
    output_dir = Path(config["output"]["output_dir"])
    output_dir.mkdir(exist_ok=True)

    # [Rest of main.py remains the same...]

if __name__ == "__main__":
    main()
```

## 5. Update pyproject.toml to include jsonschema dependency

```toml
[project]
name = "f1tenth-racetrack-generator"
version = "0.1.0"
description = "F1Tenth racetrack generator using duct-based walls"
readme = "README.md"
requires-python = ">=3.13"
dependencies = [
    "concave-hull>=0.1.2",
    "matplotlib>=3.10.8",
    "numpy>=2.4.2",
    "pyyaml>=6.0.3",
    "scipy>=1.17.0",
    "shapely>=2.1.2",
    "jsonschema>=4.20.0",  # Added for config validation
]
```

## Benefits of This Approach

1. **Single Source of Truth**: The JSON schema is the definitive documentation
2. **Automatic Validation**: Python code can validate configs before using them
3. **Self-Documenting**: Each parameter has description, constraints, and defaults
4. **IDE Support**: Many IDEs can provide autocomplete for YAML based on JSON schema
5. **Version Control**: Changes to parameters are tracked in the schema
6. **Error Messages**: Clear error messages when config is invalid

## Next Steps

1. Create the `conf/config.schema.json` file with the content above
2. Install the new dependency: `uv add jsonschema`
3. Update `main.py` to use the validator
4. Update README.md's Input Parameters section
5. Test with your existing config.yaml

This approach ensures that users always have accurate information (by reading the schema directly) and that your code always works with validated configurations.
````

---

# centerline.py 리팩토링 요청

## prompt

there was an update for config.yaml. i added config.schema.json.

so let's refactor the centerline.py file. i want to separate visualization functions to another file. for little effor of  thinking i'm thinking of file name with just 'centerline-visualizations.py'

config.shcema.json: {..}

---

# config.schema.json 참조해서 validator나 초기 과정에서 디폴트값 채워주는거 만들기

## prompt

```bash
tree
files-to-prompt conf main.py --ignore "conf/__pycache__/*" | clip
```

in current main.py code, it inquiry config.yaml with bracket indexing notation "['config']['space']['width']". but it contains failure. so there are two choices, refactoring all them to .get(K, default) pattern, or add one step that fill default values. current, there is no polyfill features in our codebase, but refering config.schema.json, it can be processed easily i think. so make new code that polyfill default values depend on config.schema.json file, and add codeline that call polyfill feature in main.py after this:

    validator = ConfigValidator(SCHEMA_FILE_PATH)

    try:
        config = validator.validate(CONFIG_FILE_PATH)
        config = config["config"]  # Extract the nested config
        print("  ✓ Configuration is valid")
    except Exception as e:
        print(f"  ✗ Configuration error: {e}")
        print(
            "\nPlease check your config.yaml against the schema at conf/config.schema.json"
        )
        return
        
i will send you current tree structures, and 4 files.
