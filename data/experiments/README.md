# Experiment Configuration Files

This directory contains experiment configuration files for the Mavnad system.

## Files

- `experiments.json` - Main experiment configuration file containing all experiment definitions

## Experiment Format

Each experiment in the JSON file contains:

- `name` - Unique identifier for the experiment
- `description` - Human-readable description
- `start_date` - Date to start the experiment (YYYY-MM-DD format)
- `start_time` - Time to start the experiment (HH:MM format)
- `duration_hours` - How long the experiment should run
- `log_interval_seconds` - How often to log sensor data (default: 120 seconds)
- `steps` - Array of experiment steps with:
  - `time` - When to execute this step (HH:MM format, relative to experiment start)
  - `fan_speed` - Fan speed percentage (0-100)
  - `inner_fan_speed` - Inner fan speed percentage (0-100)
  - `water_budget` - Water budget in seconds
  - `dampers_open` - Whether dampers should be open (true/false)
  - `description` - Description of this step

## Usage

1. Edit the `experiments.json` file to define your experiments
2. Upload the filesystem to the ESP32 using PlatformIO
3. Use serial commands to control experiments:
   - `exp list` - List available experiments
   - `exp start <name>` - Start an experiment
   - `exp status` - Check experiment status
   - `exp stop` - Stop current experiment
   - `exp pause` - Pause experiment
   - `exp resume` - Resume experiment

## Uploading to ESP32

Use PlatformIO to upload the filesystem:
```bash
platformio run --target uploadfs
```

Or in VS Code: Ctrl+Shift+P → "PlatformIO: Upload Filesystem Image"
