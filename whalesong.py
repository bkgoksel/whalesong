import pyo
from dataclasses import dataclass, field
import random
import time
import RPi.GPIO as GPIO
from gpiozero import MCP3008
import threading


def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    pin_map = {
        8: False,
        10: False,
        12: False,
        16: False,
        18: False,
        11: False,
        13: False,
        7: False,
    }

    for pin in pin_map.keys():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


# 1. --- Server Setup ---
# Initialize and boot the pyo audio server.
s = pyo.Server(duplex=0, buffersize=2048, nchnls=2)
s.deactivateMidi()
devices = pyo.pa_get_devices_infos()
device_index = -1

for d in devices:
    for idx, dev_info in d.items():
        # TODO: Update this to detect the devices we want
        if "name" in dev_info and "USB Audio Device" in dev_info["name"]:
            device_index = idx
            break
    if device_index != -1:
        break

if device_index == -1:
    device_index = 0

s.setInOutDevice(device_index)
s.boot()


@dataclass
class VoiceParam:
    base_value: float
    decay_time: float
    max_value: float = 1.0
    min_value: float = 0.0
    value: pyo.SigTo = field(init=False)
    lock: threading.RLock = field(init=False, default_factory=threading.RLock)

    def __post_init__(self):
        self.value = pyo.SigTo(
            value=self.base_value, init=self.base_value, time=self.decay_time
        )

    def boost(self, amount: float, instant: bool = False):
        with self.lock:
            if instant:
                self.value.setTime(0)
            self.value.value = min(self.value.value + amount, self.max_value)
            if instant:
                self.value.setTime(self.decay_time)


# 2. --- Voice Class for Polyphony (Whale sounds) ---
@dataclass
class WhaleVoice:
    """
    Manages a single voice in the polyphonic synth. Each voice consists
    of a sine wave oscillator modulated by an ADSR envelope and the global LFO.
    """

    root_freq: float
    base_amplitude: float = 0.01
    base_freq: pyo.SigTo = field(init=False)
    final_freq: pyo.PyoObject = field(init=False)
    lfo_ramp: pyo.SigTo = field(init=False)
    amplitude: pyo.SigTo = field(init=False)
    lfo: pyo.LFO = field(init=False)
    base_freq_lock: threading.RLock = field(default_factory=threading.RLock, init=False)
    amplitude_lock: threading.RLock = field(default_factory=threading.RLock, init=False)

    def __post_init__(self):
        self.amplitude = pyo.SigTo(self.base_amplitude, 0.01)
        self.lfo_ramp = pyo.SigTo(0.1, 20)
        self.lfo = pyo.LFO(freq=self.lfo_ramp, mul=25)
        self.base_freq = pyo.SigTo(self.root_freq, 0.5)
        self.final_freq = self.base_freq + self.lfo
        self.osc = pyo.Sine(freq=self.final_freq, mul=self.mul)
        self.decay()

    def decay(self):
        with self.mul_lock:
            self.mul.time = 0.5
            self.mul.value = 0.01
        with self.base_freq_lock:
            self.base_freq.value = self.root_freq
        threading.Timer(0.5, self.decay).start()

    def boost_amplitude(self):
        with self.mul_lock:
            self.mul.time = 0
            self.mul.value = min(0.5, self.mul.value + 0.1)

    def boost_base_freq(self):
        with self.base_freq_lock:
            self.base_freq.value = min(self.root_freq * 4, self.base_freq.value * 1.2)

    def get_output(self):
        """
        Returns the audio output of this voice.
        """
        return self.osc


# 4. --- Audio Routing and Initialization ---
freq_banks = [
    [
        100,
        400,
        800,
        1600,
    ],
    [
        150,
        600,
        1200,
        1800,
    ],
    [
        50,
        200,
        400,
        800,
    ],
    [
        300,
        450,
        1200,
        2400,
    ],
]
voice_banks = []
for bank in freq_banks:
    voice_banks.append([WhaleVoice(freq) for freq in bank])
# Mix the output of all voices together. We use a stereo mix here.
mixed_voices = pyo.Mix([v.get_output() for bank in voice_banks for v in bank], voices=2)


# Effects chain, processing the mixed signal in series.
# a. Delay for echoes. SDelay is a stereo delay with feedback filtering.
MIN_DELAY_TIME = 0.2
MAX_DELAY_TIME = 1.0
BASE_DELAY_TIME = 0.6
MIN_DELAY_FEEDBACK = 0.1
MAX_DELAY_FEEDBACK = 0.75
BASE_DELAY_FEEDBACK = 0.55
delay = pyo.Delay(mixed_voices, delay=BASE_DELAY_TIME, feedback=BASE_DELAY_FEEDBACK)

# b. LowPass filter for an "underwater" feel.
MIN_FILTER_FREQ = 100
MAX_FILTER_FREQ = 2000
BASE_FILTER_FREQ = 1200
MIN_FILTER_RES = 0
MAX_FILTER_RES = 0.5
BASE_FILTER_RES = 0.2
filter = pyo.MoogLP(delay, freq=1800, res=0.2)

# c. Reverb to create a sense of space. bal=1.0 means 100% wet signal.
reverb = pyo.Freeverb(filter, size=0.9, damp=0.7, bal=0.6)

# Apply master volume and send the final processed signal to the output.
(reverb * 0.75).out()


def activate_voices(pin):
    pin_map_list = sorted(pin_map)
    if pin == pin_map_list[0]:
        for whale in voice_banks[0]:
            whale.boost_amplitude()
    elif pin == pin_map_list[1]:
        for whale in voice_banks[0]:
            whale.boost_amplitude()
            whale.boost_base_freq()
    elif pin == pin_map_list[2]:
        for whale in voice_banks[1]:
            whale.boost_amplitude()
    elif pin == pin_map_list[3]:
        for whale in voice_banks[1]:
            whale.boost_amplitude()
            whale.boost_base_freq()
    elif pin == pin_map_list[4]:
        for whale in voice_banks[2]:
            whale.boost_amplitude()
    elif pin == pin_map_list[5]:
        for whale in voice_banks[2]:
            whale.boost_amplitude()
            whale.boost_base_freq()
    elif pin == pin_map_list[6]:
        for whale in voice_banks[3]:
            whale.boost_amplitude()
    elif pin == pin_map_list[7]:
        for whale in voice_banks[3]:
            whale.boost_amplitude()
            whale.boost_base_freq()


# 6. --- Start the Engine ---
s.start()
print("Audio engine started. Generating soundscape...")

# Use the built-in GUI to allow for easy stopping and parameter tweaking.
# Or, keep the script alive with a simple loop.
print("Press Ctrl+C in the console to stop.")

# Physical‑pin numbers you’ve wired buttons to (BOARD numbering)
delay_feedback_pot = MCP3008(channel=0)
delay_time_pot = MCP3008(channel=1)
filter_freq_pot = MCP3008(channel=2)
filter_res_pot = MCP3008(channel=3)
knob_flag = False
try:
    while True:
        for i, pin in enumerate(pin_map.keys()):
            if GPIO.input(pin) == GPIO.HIGH:
                if not pin_map[pin]:
                    activate_voices(pin)
                pin_map[pin] = True
            else:
                pin_map[pin] = False
        if knob_flag:
            delay.mul = MIN_DELAY_FEEDBACK + delay_feedback_pot.value * (
                MAX_DELAY_FEEDBACK - MIN_DELAY_FEEDBACK
            )
            delay.delay = MIN_DELAY_TIME + delay_time_pot.value * (
                MAX_DELAY_TIME - MIN_DELAY_TIME
            )
            filter.freq = MIN_FILTER_FREQ + delay_time_pot.value * (
                MAX_FILTER_FREQ - MIN_FILTER_FREQ
            )
            filter.res = MIN_FILTER_RES + delay_time_pot.value * (
                MAX_FILTER_RES - MIN_FILTER_RES
            )

except KeyboardInterrupt:
    print("\nStopping audio server...")
    s.stop()
    print("Server stopped.")
