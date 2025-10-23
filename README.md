# ROS Package [VoskRos](https://github.com/bob-ros2/voskros)

An out of the box speach to text recognizer using [Vosk speech recognition toolkit](https://alphacephei.com/vosk/).
It works offline, does not rely on external services and supports multiple languages.


## Installation Prerequisites

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install the below dependencies.

See also recommended version information on the [Vosk website](https://alphacephei.com/vosk/).

```bash
pip3 install vosk
pip3 install sounddevice
# maybe these are also needed
sudo apt-get install libportaudio2
sudo apt-get install libasound-dev
```

## Setup Package ##

```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/voskros.git
cd ..
colcon build
. install/setup.bash
```

## Usage

```bash
# get device list (for microphone input mode)
ros2 run voskros vosk -l

# run node with default parameter (microphone input)
ros2 run voskros vosk

# run via launch file with specific model
ros2 launch voskros voskros.launch.yaml model:=fr

# run via launch and override result output topic
ros2 launch voskros voskros.launch.yaml result:=/tts_topic

# run with multiple audio topics (new feature)
ros2 launch voskros voskros_multi.launch.yaml audio_topics:="['/audio/mic1', '/audio/mic2']"

# run with custom models directory
ros2 launch voskros voskros.launch.yaml models_dir:=/custom/path/to/models
```

### Multi-Microphone Configuration

You can configure multiple audio topics using a YAML configuration file. See `config/audio_topics.yaml` for an example:

```yaml
audio_topics:
  - /audio/microphone1
  - /audio/microphone2

samplerate: 16000
model: en-us
```

Then launch with the configuration:

```bash
ros2 launch voskros voskros_multi.launch.yaml
```

**Note:** When using multiple audio topics, the node subscribes to ROS topics instead of directly reading from microphone devices. You'll need to publish audio data to these topics using another node (e.g., audio_capture or a custom audio publisher).

For detailed documentation on multi-topic usage, see [docs/MULTI_TOPIC_USAGE.md](docs/MULTI_TOPIC_USAGE.md).

## Helper audio_publisher

For testing the multi-topic feature, an example audio publisher script is included:

```bash
# Publish a WAV file to an audio topic
ros2 run voskros audio_publisher <audio_file.wav> <topic_name>

# Example: Publish test.wav to /audio/microphone1
ros2 run voskros audio_publisher test.wav /audio/microphone1
```

The audio file should be in WAV format with the following specifications:
- Format: PCM 16-bit signed integer
- Channels: Mono (1 channel)  
- Sample Rate: 16000 Hz (or as configured in the node)

## Models

The package now loads models from a local `models/` directory in the package. During first startup, if a model is not found locally, it will be automatically downloaded from https://alphacephei.com/vosk/models.

To manually download and install models:

```bash
# Navigate to the models directory
cd /path/to/voskros/models

# Download a model (example: English US)
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip

# Extract the model
unzip vosk-model-small-en-us-0.15.zip
```

Find more models and languages here: https://alphacephei.com/vosk/models

## Node VOSK

### Node Parameter

> ~device (string, default: "")\
Device to use as input microphone. Leave it empty to use the default input. Note: If audio_topics is specified, this parameter is ignored.

> ~model (string, default: "en-us")\
Model to be used. The model will be loaded from the local models/ directory.

> ~samplerate (int, default: 16000)\
Sample rate to use for audio processing.

> ~audio_topics (string array, default: [])\
List of ROS audio topics to subscribe to. If specified, the node will subscribe to these topics instead of using a microphone device. Each topic should publish audio data as Int8MultiArray messages.

> ~models_dir (string, default: "")\
Path to the models directory. If not specified, defaults to the models/ folder in the package.

### Published Topics

> ~result (std_msgs/String)\
Detected final result text with confidence score and source. Format: "text | confidence: 0.95 | source: /audio/mic1"

### Subscribed Topics

> ~audio/topic_name (std_msgs/Int8MultiArray)\
Audio data from configured topics (when using audio_topics parameter). Audio should be PCM 16-bit, mono, at the configured sample rate.

### Services

> ~set_grammar (srv/SetGrammar)\
Set Vosk grammar list to only allow certain words. See also [srv/SetGrammar.srv](srv/SetGrammar.srv) for details.

## Node PROMPTER

This very simple ROS Node can be used to receive an input command (e.g. from Vosk). If this command matches with one of the rules in the config file the according command will be executed.

### Usage

```bash
# start Vosk if not yet running
# in order to work a microphone must be connected or another input device must be configured
ros2 launch voskros voskros.launch.yaml

# if needed reduce the grammar according to the config in prompter.yaml
# setting the grammar greatly improves the STT detection
ros2 run voskros set_grammar.sh <path>/prompter.yaml /stt

# start prompter node and wait for commands to be executed
ros2 run voskros prompter --ros-args \
    -r input:=/stt/result \
    -r __ns:=/stt \
    -p yaml:=<path>/prompter.yaml

```

### Node Parameter

> ~yaml (string, default: "")\
Path to a YAML file containing the prompter configuration.\
See config directory for an example.

> ~cwd (string, default: "")\
Working directory where to start programms from the configuration. 

### Subscribed Topics

> ~input (std_msgs/String)\
Input topic for incommning commands to identify.

### Published Topics

> ~output (std_msgs/String)\
Output topic containing stdout produced by the called program.

## Helper set_grammar.sh
```bash
$ ros2 run voskros set_grammar.sh
Usage: set_grammar.sh <prompter config> [<namespace>]
Produces a JSON Grammar array for vosk from a prompter config yaml.
It calls afterwards the vosk node service set_grammar if a namespace is provided.
The Grammar can also be configured using a vosk node parameter.
```
