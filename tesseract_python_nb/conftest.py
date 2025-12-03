"""pytest configuration - auto-load environment variables from .env"""
import os
from pathlib import Path

def pytest_configure(config):
    """Load .env file before tests run."""
    env_file = Path(__file__).parent / ".env"
    if env_file.exists():
        with open(env_file) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, _, value = line.partition("=")
                    # Handle ${VAR} expansion
                    if "${" in value:
                        import re
                        for match in re.finditer(r'\$\{(\w+)\}', value):
                            var_name = match.group(1)
                            var_value = os.environ.get(var_name, "")
                            value = value.replace(match.group(0), var_value)
                    os.environ[key.strip()] = value.strip()
