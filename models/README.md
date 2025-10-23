# Vosk Models Directory

This directory contains the Vosk speech recognition models used by the voskros package.

## Automatic Download

When you specify a model name in the launch file or configuration, the voskros node will automatically download it from the official Vosk website (https://alphacephei.com/vosk/models) if it's not already present in this directory.

## Manual Download

If automatic download fails (e.g., due to network restrictions), you can manually download models:

1. Visit https://alphacephei.com/vosk/models
2. Download the desired model (e.g., `vosk-model-small-en-us-0.15.zip`)
3. Extract the ZIP file into this directory
4. The structure should be: `models/vosk-model-small-en-us-0.15/` containing model files

## Supported Languages

The voskros package supports various languages through different models:

- `en-us` or `en`: English (US) - Small model (~50MB)
- `fr`: French - Small model
- `de`: German - Small model
- `es`: Spanish - Small model
- `it`: Italian - Small model
- `ru`: Russian - Small model
- `zh`: Chinese - Small model

For better accuracy, you can download larger models from the Vosk website.

## Model Structure

Each model directory should contain:
- `am/` - Acoustic model directory
- `graph/` - Language model directory
- `conf/mfcc.conf` - Feature configuration
- Other model-specific files

## Usage

Specify the model name in your launch file:

```bash
ros2 launch voskros voskros.launch.yaml model:=en-us
```

Or in the configuration file:

```yaml
model: en-us
```

The node will look for the model in this directory and download it if necessary.
