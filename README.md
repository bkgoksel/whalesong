# Whale Player for the Gourd!

## Installation

Install libraries needed for `pyo` and `jackd`:

```
sudo apt-get update
sudo apt-get install libportaudio2 libsndfile1 liblo-dev libportmidi-dev jackd2
```

Use `uv` as all the cool kids now do:

```
curl -LsSf https://astral.sh/uv/install.sh | sh 
```

```
uv sync
uv venv
```

```
sudo .venv/bin/python main.py
```


## Configuring the merged audio device

Make sure the audio cards you want to use are plugged in.
Run the following command to list available devices:

```
aplay -l
```


Note the card #s of the devices you want to merge (ie where it says `card 0: Headphones` you want that number)
Edit `asound.conf` and update the device numbers of the two virtual devices defined on the top to be your devices.
Move `asound.conf` under `/etc` to update your system audio config.

```
sudo cp asound.conf /etc/asound.conf
```

## Installation as a system service on a RasPi

Update the last line of `start.sh` and replace `/home/kerem/whalesong` with the absolute path to *this* directory on your system.

```
sudo cp whalesong.service /etc/systemd/system/
sudo systemctl enable whalesong.service
sudo systemctl start whalesong.service
```


Need to run as `sudo` because the `keyboard` lib requires it on linux!
