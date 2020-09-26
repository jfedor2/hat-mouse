# Hat mouse

This is Arduino code for an "air mouse" (or "head mouse") type of input device. It works as a standard Bluetooth mouse and is compatible with Windows, Linux and Mac with no additional software running on the computer.

[Demo video.](https://www.youtube.com/watch?v=aJb6M_2izWQ)

I'm using Adafruit's [Feather nRF52840 Sense](https://www.adafruit.com/product/4516) board, but it should also run on their other nRF52 boards and modifying it to use a different set of sensors should be pretty straightforward.

The way it works is it reads accelerometer, gyroscope and magnetometer data, puts it through a sensor fusion algorithm to get orientation and then uses the yaw and pitch values to position the mouse cursor on the screen. It uses absolute cursor positioning instead of the most common relative mode, hence the custom HID report descriptor.

Cursor re-centering is done by pressing the "user switch" button on the board.

On the demo video I'm using it with a separate USB foot switch for mouse clicks. It could also conceivably be combined with some kind of sip-and-puff switch, though I'm not showing one here.
