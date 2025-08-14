import argparse
import pyo
from dataclasses import dataclass, field, InitVar
import RPi.GPIO as GPIO

import keyboard
import threading
import time
import random
from typing import Callable


class Inputs:
    def listen(self):
        raise NotImplementedError


@dataclass
class GPIOButtonInputs(Inputs):
    """
    Simple GPIO button input listener. Pass it a `pins` dictionary of GPIO input pins to callbacks.
    Every time voltage at a pin goes from low to high (ie a button press) it calls the callback.
    Does not trigger the callback continuously, only on low->high voltage changes.
    `listen()` method is blocking method that infinite loops until killed. Start in a thread if needed.
    """

    pins: dict[int, Callable[[], None]]
    pin_activations: dict[int, bool] = field(init=False)

    def __post_init__(self) -> None:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.pin_activations = {p: False for p in self.pins}

    def listen(self):
        while True:
            for pin, callback in self.pins.items():
                if GPIO.input(pin) == GPIO.HIGH:
                    if not self.pin_activations[pin]:
                        callback()
                    self.pin_activations[pin] = True
                else:
                    self.pin_activations[pin] = False
            time.sleep(0.01)


@dataclass
class KeyboardInputs(Inputs):
    """
    Simple Keyboard input listener. Uses https://github.com/boppreh/keyboard
    Given a mapping from keys to callbacks, sets up listeners to trigger the callbacks.
    """

    keys: dict[str, Callable[[], None]]

    def __post_init__(self):
        for key, callable in self.keys.items():
            keyboard.add_hotkey(
                key,
                callable,
                timeout=0.1,  # defaults to 1 (second), allow more frequent re-presses
            )

    def listen(self) -> None:
        keyboard.wait()


@dataclass
class DecayingParameter:
    """
    Wrapper for a PYO paramater that can be "boosted" via external events, but always decays down to its base value.
    To use in PYO sound modules, pass my_parameter.value where you'll use it. (eg lfo = pyo.LFO(decaying_freq.value, mul=1))
    """

    base_value: float  # base value that this always decays to.
    decay_time: float  # time it takes for this to decay back to its base value after boosting is done.
    wraparound: bool = False
    max_value: float = (
        1.0  # maximum value you can boost up to, gets clamped at this value.
    )
    min_value: float = 0.0  # minimum value, in case you have a negative "boost".
    default_boost: float = 0.1  # default amount for a boost call, can be overriden.
    target_value: float = field(
        init=False
    )  # For internal use, tracks current "target value" this is ramping towards.
    value: pyo.Sig = field(
        init=False
    )  # The actual pyo.Sig object that can be used as a signal input.
    ramping: bool = field(
        default=False, init=False
    )  # For internal use. Thread-safety variable to track when the value is still ramping up to a boosted value.
    ramping_timer: threading.Timer | None = field(
        init=False, default=None
    )  # For internal use. Timer thread to set ramping flag to false.
    lock: threading.RLock = field(
        init=False, default_factory=threading.RLock
    )  # For internal use. Lock for thread safety.

    def __post_init__(self):
        self.target_value = self.base_value
        self.value = pyo.Sig(value=self.base_value)
        self.decay()

    def decay(self):
        with self.lock:
            if not self.ramping and self.target_value != self.base_value:
                self.value.value = pyo.SigTo(
                    value=self.base_value,
                    init=self.target_value,
                    time=self.decay_time,
                )
                self.target_value = self.base_value
        threading.Timer(1, self.decay).start()

    def boost(self, boost_amount: float | None = None, ramp_time: float = 0):
        boost_amount = boost_amount or self.default_boost
        with self.lock:
            current_value = self.value.get()
            new_value = current_value + boost_amount
            if new_value > self.max_value:
                if self.wraparound:
                    self.target_value = self.min_value + (new_value - self.max_value)
                else:
                    self.target_value = self.max_value
            else:
                self.target_value = new_value
            if ramp_time > 0:
                self.value.value = pyo.SigTo(
                    value=self.target_value,
                    init=current_value,
                    time=ramp_time,
                )
                self.ramping = True

                def set_ramping():
                    with self.lock:
                        self.ramping = False

                if self.ramping_timer is not None:
                    self.ramping_timer.cancel()
                self.ramping_timer = threading.Timer(ramp_time, set_ramping)
                self.ramping_timer.start()
            else:
                self.value.value = self.target_value


@dataclass
class WhaleVoice:
    freqs: InitVar[list[int]]
    freq_boost: DecayingParameter
    amplitude: DecayingParameter
    freq_modulation: DecayingParameter
    pan_point: DecayingParameter
    freq_modulation_rate: float = 0.2
    num_pan_channels: InitVar[int] = 2
    lfo: pyo.LFO = field(init=False)
    pan_lfo: pyo.Sine = field(init=False)

    def __post_init__(self, freqs: list[int], num_pan_channels: int = 2):
        self.pan_lfo = pyo.Sine(freq=0.5, mul=0.05)
        self.lfo = pyo.LFO(
            freq=self.freq_modulation_rate, mul=self.freq_modulation.value
        )
        self.oscs = [
            pyo.Sine(freq=freq + self.freq_boost.value + self.lfo) for freq in freqs
        ]
        self.mix = pyo.Mix(self.oscs, voices=1, mul=self.amplitude.value)
        self.pan = pyo.Pan(
            self.mix, outs=num_pan_channels, pan=self.pan_point.value, spread=0.05
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-name", "-d", type=str, default="Headphones")
    parser.add_argument(
        "--audio-driver", choices=["jack", "portaudio"], default="portaudio"
    )
    parser.add_argument("--n-channels", "-n", type=int, default=2)
    parser.add_argument(
        "--input-type", choices=["gpio", "keyboard"], default="keyboard"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # 1. --- Server Setup ---
    # Initialize and boot the pyo audio server.
    s = pyo.Server(
        duplex=0, buffersize=1024, audio=args.audio_driver, nchnls=args.n_channels
    )
    s.deactivateMidi()
    if args.audio_driver != "jack":
        devices = pyo.pa_get_devices_infos()
        device_index = -1
        for d in devices:
            for idx, dev_info in d.items():
                # TODO: Update this to detect the devices we want
                if "name" in dev_info and args.device_name in dev_info["name"]:
                    device_index = idx
                    break
            if device_index != -1:
                break
        if device_index == -1:
            raise ValueError(f"Device {args.device_name} not found on system")
        s.setInOutDevice(device_index)
    s.boot()
    s.start()

    # 4. --- Audio Routing and Initialization ---
    FREQ_DECAY_TIME = 2
    FREQ_MOD_DECAY_TIME = 0.25
    FREQ_MOD_BOOST_AMT = 50
    AMP_DECAY_TIME = 1
    AMP_BOOST_AMT = 0.1
    freq_banks = [
        [
            50,
            200,
            400,
            800,
        ],
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
            200,
            800,
            1600,
            2400,
        ],
        [
            300,
            450,
            1200,
            2400,
        ],
        [
            350,
            700,
            1400,
            2800,
        ],
        [
            400,
            800,
            1600,
            2800,
        ],
        [
            450,
            900,
            1400,
            2400,
        ],
    ]
    voices = []
    for bank_idx, bank in enumerate(freq_banks):
        freq_boost = DecayingParameter(
            base_value=0,
            decay_time=FREQ_DECAY_TIME,
            max_value=2000,
            min_value=0,
            default_boost=FREQ_MOD_BOOST_AMT,
        )
        amplitude = DecayingParameter(
            base_value=0.01,
            decay_time=AMP_DECAY_TIME,
            max_value=0.65,
            min_value=0.0,
            default_boost=AMP_BOOST_AMT,
        )
        freq_modulation = DecayingParameter(
            base_value=25,
            decay_time=FREQ_MOD_DECAY_TIME,
            max_value=500,
            min_value=25,
            default_boost=FREQ_MOD_BOOST_AMT,
        )
        pan_point = DecayingParameter(
            decay_time=0.5,
            wraparound=True,
            base_value=bank_idx * (1 / len(freq_banks)),
            max_value=1.0,
            min_value=0.0,
            default_boost=0.5,
        )
        voices.append(
            WhaleVoice(
                freqs=bank,
                freq_boost=freq_boost,
                amplitude=amplitude,
                freq_modulation=freq_modulation,
                freq_modulation_rate=0.05 + random.random(),
                pan_point=pan_point,
                num_pan_channels=args.n_channels,
            )
        )
    # Mix the output of all voices together. We use a stereo mix here.
    mixed_voices = pyo.Mix([v.pan for v in voices], voices=4)

    # Effects chain, processing the mixed signal in series.
    delay = pyo.Delay(mixed_voices, delay=0.6, feedback=0.5)
    post_delay = mixed_voices * 0.6 + delay * 0.25
    filter = pyo.MoogLP(post_delay, freq=1800, res=0.2)
    reverb = pyo.Freeverb(filter, size=0.9, damp=0.7, bal=0.3)
    (reverb * 0.7).out()
    # (mixed_voices * 0.75).out()

    # 6. --- Start the Engine ---

    def trigger(button_index):
        voice = button_index // 2
        whale = voices[voice]
        if button_index % 2 == 1:
            whale.amplitude.boost(ramp_time=0.2, boost_amount=0.2)
            whale.freq_boost.boost(ramp_time=1)
            whale.pan_point.boost(ramp_time=0.25)
        else:
            whale.amplitude.boost(ramp_time=0.1, boost_amount=0.4)

    gpio_inputs: Inputs = GPIOButtonInputs(
        pins={
            7: lambda: trigger(0),
            8: lambda: trigger(1),
            10: lambda: trigger(2),
            11: lambda: trigger(3),
            12: lambda: trigger(4),
            13: lambda: trigger(5),
            16: lambda: trigger(6),
            18: lambda: trigger(7),
        }
    )

    keyboard_inputs: Inputs = KeyboardInputs(
        keys={
            "q": lambda: trigger(0),
            "e": lambda: trigger(2),
            "w": lambda: trigger(1),
            "r": lambda: trigger(3),
            "t": lambda: trigger(4),
            "y": lambda: trigger(5),
            "a": lambda: trigger(6),
            "s": lambda: trigger(7),
            "d": lambda: trigger(8),
            "f": lambda: trigger(9),
            "g": lambda: trigger(10),
            "h": lambda: trigger(11),
            "z": lambda: trigger(12),
            "x": lambda: trigger(13),
            "c": lambda: trigger(14),
            "v": lambda: trigger(15),
        }
    )

    if args.input_type == "gpio":
        inputs = gpio_inputs
    elif args.input_type == "keyboard":
        inputs = keyboard_inputs
    else:
        raise ValueError(f"Input type {args.input_type} not supported")

    print("Audio engine started. Generating soundscape...")
    print("Press Ctrl+C in the console to stop.")
    try:
        inputs.listen()
    except KeyboardInterrupt:
        print("\nStopping audio server...")
        s.stop()
        print("Server stopped.")


if __name__ == "__main__":
    main()
