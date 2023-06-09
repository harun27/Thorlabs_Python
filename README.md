## Information
- This is a Python Library for Thorlabs devices. I just implemented the LTS150 Translation Stage
- To get more information about the library I used, read: https://github.com/yaq-project/thorlabs-apt-protocol

## Usage
- Look at the ``main.py`` file, on how to use this. To get more info about the methods of the classes, see in ``Thorlabs.py``

## Windows Only: Enable Virtual COM Port
On Windows, the virtual serial communications port (VCP) may need to be enabled in the driver options for the USB interface device. First, open the Windows Device Manager. If plugging in the controller causes a new COM device to appear under the “Ports (COM & LPT)” section, then there is nothing more to do. If a new COM device does not appear, then find the controller device under “Universal Serial Bus Controllers”, it may be called “Thorlabs APT Controller” or similar (see what new device appears when plugging in the controller). Right-click->Properties->Advanced tab, check the “Load VCP” box, then OK out of the dialog back to the device manager. Unplug and re-plug the USB connection to the controller, and ensure than a new COM device now appears.
