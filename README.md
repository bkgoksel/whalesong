# Whale Player for the Gourd!

## Installation

Use `uv` as all the cool kids now do:

```
curl -LsSf https://astral.sh/uv/install.sh | sh 
```

```
uv venv
```

```
sudo .venv/bin/python main.py
```

## Installation as a system service on a RasPi

```
sudo cp whalesong.service /etc/systemd/system/
sudo systemctl enable whalesong.service
sudo systemctl start whalesong.service
```


Need to run as `sudo` because the `keyboard` lib requires it on linux!
