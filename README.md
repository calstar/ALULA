# ALULA (LE2)
 Alula/ LE2 Hot-fire and Flight Vehicle Electronics Control


## Summary
**DAQboard.ino** file commands the ESP32 that reads data from sensors and commands from the COM Board, and sends data in a package to the COM Board.

**COMBoard.ino** file commands the ESP32 that receives data in a package from the DAQ  Board, sends commands to the DAQ Board, and sends data to MATLAB for plotting and data-saving purposes through the serial port.

**liveplot.m** is a MATLAB file that can receive data from the COM Board, create live plots, and save the data.
  1. **Change the test file name according to the trial number in the liveplot.m file.**
  2. Click "Run" to start the code, and close the plotting window to stop the code. Test data will be automatically stored in a spreadsheet under a folder called "**Test_Data_yyyy-mm-dd**."

## ESP32 MAC Addresses
- DAQ Breadboard:{0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
- COM Breadboard:{0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C};
- DAQ Protoboard:








## Software requirement
**Ctrl click (Windows) or Cmd click the following links to open them in new tabs**
1. Arduino (https://www.arduino.cc/en/software)
2. MATLAB (https://software.berkeley.edu/matlab®)
    - To control the serial port, you need to install the [**Instrument Control Toolbox**](https://www.mathworks.com/products/instrument.html).
3. (optional) An external code editing software such as Atom (https://atom.io) or Visual Studio (https://visualstudio.microsoft.com/downloads/).
    - Arduino IDE and MATLAB are sufficient for all the coding
4. (recommended) GitHub Desktop (https://desktop.github.com). You can pull, push, and edit files with ease. **Step 3 required if you would like to download GitHub Desktop as the software requires an external editor.**


## How to update
### Using Github Desktop
1. Make sure you have access to this project
    - Check with you administrators
2. Go to the Github app, sign in, and click on the top left corner. You should see a pop-up window with an "Add" button. Select that button and choose "Clone repository."
3. Search in "Filter you repositories" using the keyword "ellie-ground." Select the correct repository and click "Clone."
    - If you want this cloned repository to be in a different folder, change the "Local Path."
4. After you have edited and saved the file in the local repository, GitHub Desktop will detect changes and prompt you to commit. Make sure to double check your changes before committing.
5. On the bottom left corner, put down a short summary that describes what you have changed, then click "Commit to **main**."
6. In the middle of the screen (or on the top banner) you will see "Push commits to the origin remote." If you are sure that the commit is ready, click "Push origin."
6. Check the project online and see if the changes have been reflected.


### Using Git
Please refer to https://rocketry.gitbook.io/public/tutorials/avionics/git-and-workflow for more information.


## Helpful resources
### ESP32-DevkitC/ ESP32-Devkit-v4 (for PCB application - antenna sticks out of board)
Getting started

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html

Pinout reference

![esp32 devkitC pinout](https://github.com/calstar/ellie-ground/blob/main/Misc_resources/esp32-devkitC-v4-pinout.png?raw=true "ESP32 DevkitC Pinout")

### ESP32-WROOM/ ESP32-Devkit-v1 (for breadboard/protoboard application - antenna does NOT stick out of board)

Pinout reference
![esp32 wroom pinout](https://www.mischianti.org/wp-content/uploads/2020/11/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png)

### ESP32-HUZZAH (for breadboard/protoboard application)

Pinout reference
![esp32 devkitC pinout](https://github.com/calstar/ellie-ground/blob/main/Misc_resources/esp32-huzzah-pinout.png?raw=true "ESP32 HUZZAH Pinout")

### Pressure transducer color code
Red - E+

Black - E-

Green - A-

White - A+

Other possible combinations: Blue E+; Red E-; Green A-; Black A+
