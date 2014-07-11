# Launch Oeta on startup based upon GPIO 9 (server or standalone mode). Monitor for power off requests

# Setup fbtft_device, change framebuffer and startx

export DISPLAY=:0.0
export FRAMEBUFFER=/dev/fb1

if [ $(tty) == /dev/tty1 ]; then

# Set server switch
gpio mode 9 in
gpio mode 9 up

# Set volume
amixer set PCM 80

# Start X
echo "Starting X..."

if [ $(gpio read 9) == 1 ]; then
	exec startx ~/.xinitrc_server -- -nocursor -nolisten tcp &
else
	exec startx ~/.xinitrc_normal -- -nocursor -nolisten tcp &
fi

echo "Initiating power switch listener..."

# Power switch
# This is the GPIO pin connected to the lead on switch labeled OUT
GPIOpin1=4

# This is the GPIO pin connected to the lead on switch labeled IN
GPIOpin2=17

echo "$GPIOpin1" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio$GPIOpin1/direction
echo "$GPIOpin2" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio$GPIOpin2/direction
echo "1" > /sys/class/gpio/gpio$GPIOpin2/value
while [ 1 = 1 ]; do
power=$(cat /sys/class/gpio/gpio$GPIOpin1/value)
if [ $power = 0 ]; then
sleep 1
else
echo "piRadio: Shutting down..."
poweroff
fi
done


fi
