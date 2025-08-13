#!/bin/sh

echo "--- JACK Audio Server Restart ---"

# Check if a jackd process is currently running.
# pgrep returns 0 if a process is found, 1 if not.
if pgrep -x "jackd" >/dev/null; then
  echo "Found a running jackd instance. Terminating it now..."
  # killall sends a signal to all processes with the given name.
  # The -w flag makes it wait until the processes have terminated.
  killall -w jackd
  echo "jackd process terminated."
else
  echo "No running jackd process found. Proceeding to start a new one."
fi

# A brief pause to ensure system resources are freed up.
sleep 1

echo "Starting a new jackd process in the background..."

JACK_NO_AUDIO_RESERVATION=1 jackd -d alsa -d fourplay -r 44100 -p 1024 &

# Give jackd a couple of seconds to initialize.
sleep 2

# Final check to confirm the new process has started.
if pgrep -x "jackd" >/dev/null; then
  echo "✅ Success! JACK server is now running with PID: $(pgrep -x "jackd")"
else
  echo "❌ Error: Failed to start jackd. Check your parameters or system logs for more info."
fi

echo "-------------------------------"
# TODO: Update the path here to match the absolute path of the repo on your system.
/home/kerem/whalesong/.venv/bin/python /home/kerem/whalesong/main.py
