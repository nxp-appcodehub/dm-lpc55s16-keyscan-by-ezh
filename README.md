# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## EZH implements keyscan function in lpc5516
This demo introduces a method to implement 4x4 key value scan by using EZH in LPC55.

EZH, as a co-processor of LPC55, can quickly access IO. For mechanical repetitive tasks like keyscan, EZH provides high and low levels for each row of the keyboard, then quickly reads the level of each column, and finally determines which button is pressed. It is very suitable for keyboard applications.


#### Boards: LPCXpresso55S16
#### Categories: Industrial, HMI
#### Peripherals: GPIO, UART
#### Toolchains: MCUXpresso IDE

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [Support](#step5)
7. [Release Notes](#step6)

## 1. Software<a name="step1"></a>
- [MCUXpresso IDE V11.9.0 or later](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).
- SDK_2_15_000_LPCXpresso55S16
- MCUXpresso for Visual Studio Code: This example supports MCUXpresso for Visual Studio Code, for more information about how to use Visual Studio Code please refer [here](https://www.nxp.com/design/training/getting-started-with-mcuxpresso-for-visual-studio-code:TIP-GETTING-STARTED-WITH-MCUXPRESSO-FOR-VS-CODE).

## 2. Hardware<a name="step2"></a>
- Type-C USB cable
- LPCXpresso55S16
- Personal Computer
- Digilent PmodKYPD
- USB to TLL board

## 3. Setup<a name="step3"></a>
### 3.1 Step 1

Connect the PmodKYPD to the PMOD header on LPCXpresso55S16, as shown below:

![hardware](./images/hardware.png)

### 3.2 Step 2

- Import the project to MCUXpresso IDE.

1. Open MCUXpresso IDE, in the Quick Start Panel, choose **Import from Application Code Hub**.

   ​	![](images/import_project_1.png)

2. Enter the demo name in the search bar.

   ![](images/import_project_2.png) 

3. Click **Copy GitHub link**, MCUXpresso IDE will automatically retrieve project attributes, then click **Next>**.

   ​	![](images/import_project_3.png)

4. Select **main** branch and then click **Next>**, Select the MCUXpresso project, click **Finish** button to complete import.

   ​	![](images/import_project_4.png)

- Connect the micro USB cable between the PC host and the USB port (J1) on the board.
- Short JP9 with jumper.   
- connect RX pin and ground pin of USB to TLL board with JP3 on LPCXpresso55S16.
- Open a serial terminal on PC for the serial device with these settings:
  - 115200 baud rate
  - 8 data bits
  - No parity
  - One stop bit
  - No flow control
- Compile and download to the board.
- Reset and run.

## 4. Results<a name="step4"></a>
Push down the keys on PmodKYPD board, the value will be shown on the serial terminal as below:

Button 3  is pressed 
Button 9  is pressed 
Button E  is pressed 
Button D  is pressed 
Button B  is pressed 
Button A  is pressed

## 5. Support<a name="step5"></a>
#### Project Metadata
<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-LPCXPRESSO55S16-blue)](https://github.com/search?q=org%3Anxp-appcodehub+LPCXpresso55S16+in%3Areadme&type=Repositories)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-INDUSTRIAL-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+industrial+in%3Areadme&type=Repositories) [![Category badge](https://img.shields.io/badge/Category-HMI-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+hmi+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+gpio+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-UART-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+uart+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://github.com/search?q=org%3Anxp-appcodehub+mcux+in%3Areadme&type=Repositories) [![Toolchain badge](https://img.shields.io/badge/Toolchain-VS%20CODE-orange)](https://github.com/search?q=org%3Anxp-appcodehub+vscode+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 6. Release Notes<a name="step6"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code Hub        | July 16<sup>th</sup> 2024 |

