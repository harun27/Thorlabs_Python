# Information
- This is a Python Library for Thorlabs devices. I just implemented the LTS150 Translation Stage
- To get more information about the library I used, read: https://github.com/yaq-project/thorlabs-apt-protocol

## Windows Only: Enable Virtual COM Port
On Windows, the virtual serial communications port (VCP) may need to be enabled in the driver options for the USB interface device. First, open the Windows Device Manager. If plugging in the controller causes a new COM device to appear under the “Ports (COM & LPT)” section, then there is nothing more to do. If a new COM device does not appear, then find the controller device under “Universal Serial Bus Controllers”, it may be called “Thorlabs APT Controller” or similar (see what new device appears when plugging in the controller). Right-click->Properties->Advanced tab, check the “Load VCP” box, then OK out of the dialog back to the device manager. Unplug and re-plug the USB connection to the controller, and ensure than a new COM device now appears.


# Usage
## Initialization
- To use this code, you first have to initalize the device stage with the apt software
- To do that, just download the APT Software from https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=1
- Then open the program `APT Config`
- There you will have to choose the motor, channel and the stage (f.e. LTS150 150mm Stage) and click on ``Add/Change Stage Association`` to apply the changes

## Test it
- Look at the examples folder. There you will find a sample application for every supported device.

# Supported Devices
- LTS150
- LTS300