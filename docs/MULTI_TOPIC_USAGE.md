# Multi-Topic Audio Support - Usage Guide

This guide explains how to use the new multi-topic audio support feature in voskros.

## Overview

The voskros package now supports:
1. Loading models from a local `models/` directory
2. Subscribing to multiple ROS audio topics simultaneously
3. Publishing final results with confidence scores

## Quick Start

### Option 1: Using Microphone Input (Original Behavior)

```bash
# Run with default microphone
ros2 run voskros vosk

# Run with specific microphone device
ros2 run voskros vosk --ros-args -p device:="USB Audio Device"

# Run via launch file
ros2 launch voskros voskros.launch.yaml model:=en-us
```

### Option 2: Using ROS Audio Topics (New Feature)

1. **Configure audio topics** in `config/audio_topics.yaml`:

```yaml
audio_topics:
  - /audio/microphone1
  - /audio/microphone2

samplerate: 16000
model: en-us
```

2. **Launch the node** with audio topic support:

```bash
ros2 launch voskros voskros_multi.launch.yaml audio_topics:="['/audio/mic1', '/audio/mic2']"
```

3. **Publish audio data** to the configured topics using a compatible audio publisher.

## Audio Topic Format

Audio topics should publish messages of type `std_msgs/Int8MultiArray` with the following format:

- **Encoding**: PCM 16-bit signed integer, little-endian
- **Channels**: Mono (1 channel)
- **Sample Rate**: As configured (default: 16000 Hz)
- **Message Data**: Raw audio bytes as int8 array

### Example Audio Publisher

The package includes an example audio publisher for testing:

```bash
# Record a test audio file (using arecord or similar)
arecord -f S16_LE -c 1 -r 16000 -d 5 test.wav

# Publish the audio file to a topic
ros2 run voskros audio_publisher test.wav /audio/microphone1
```

## Output Format

The result topic now publishes messages in the following format:

```
<text> | confidence: <0.00-1.00> | source: <topic_name>
```

Example:
```
hello world | confidence: 0.95 | source: /audio/microphone1
```

### Subscribing to Results

```bash
# View results from all sources
ros2 topic echo /stt/result

# Filter results from specific source
ros2 topic echo /stt/result | grep "microphone1"
```

## Model Management

### Automatic Download

When you specify a model that doesn't exist locally, it will be automatically downloaded:

```bash
ros2 launch voskros voskros.launch.yaml model:=fr
```

### Manual Download

For environments without internet access:

1. Download model from https://alphacephei.com/vosk/models
2. Extract to `models/` directory:
   ```bash
   cd /path/to/voskros/models
   wget https://alphacephei.com/vosk/models/vosk-model-small-fr-0.22.zip
   unzip vosk-model-small-fr-0.22.zip
   ```

3. Use the model:
   ```bash
   ros2 launch voskros voskros.launch.yaml model:=vosk-model-small-fr-0.22
   ```

### Custom Models Directory

To use a custom location for models:

```bash
ros2 launch voskros voskros.launch.yaml models_dir:=/custom/path/to/models
```

## Troubleshooting

### Model Download Fails

If automatic download fails:
1. Check internet connectivity
2. Manually download from https://alphacephei.com/vosk/models
3. Extract to `models/` directory
4. Ensure correct permissions

### No Audio Received

1. Verify audio topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic hz /audio/mic1
   ```

2. Check audio format matches configuration
3. Verify sample rate is correct (default: 16000 Hz)

### Low Recognition Accuracy

1. Use a larger, more accurate model
2. Set grammar constraints for your use case
3. Ensure good audio quality (reduce background noise)
4. Check microphone placement and quality

## Additional Resources

- Vosk Models: https://alphacephei.com/vosk/models
- Vosk API Documentation: https://alphacephei.com/vosk/
- Package Repository: https://github.com/alobo01/voskros
