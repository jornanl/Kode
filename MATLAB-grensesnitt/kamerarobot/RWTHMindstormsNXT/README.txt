* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
* Readme file for RWTH - Mindstorms NXT Toolbox for MATLAB  *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - *
*           Version 2.00 - September 26th, 2008             *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



CONTENTS
---------

1. Copyrights / License
2. System Requirements
3. Quick Start
4. Installation Guide
5. First Bluetooth Connection
6. Acknowledgements
7. Websites



1. COPYRIGHTS / LICENSE
-----------------------

The RWTH - Mindstorms NXT Toolbox is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

The RWTH - Mindstorms NXT Toolbox is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with the RWTH - Mindstorms NXT Toolbox. If not, see www.gnu.org/licenses.



2. SYSTEM REQUIREMENTS
----------------------

- Operating system: Windows or Linux
- MATLAB Version 7.4 (R2007a) or higher
- LEGO® Mindstorms NXT building kit (e.g. Education Kit)
- LEGO® Mindstorms NXT firmware v1.05 (recommended) 
  OR a firmware compatible to the NXT's direct command interface
- Bluetooth: Bluetooth 2.0 adapter recommended model by LEGO® 
  (e.g. AVM BlueFRITZ! USB) supporting the serial port profile (SPP)  
- USB: 
    Windows: Official MINDSTORMS NXT Driver "Fantom", v1.02 or better 
    Linux: libusb or libusb-dev  


*** Bluetooth:

The SPP (serial port profile) means that a virtual serial port is installed, which maps all data to the Bluetooth interface. You can verify this under Windows inside the Device Manager, where an additional COM-Port should be visible. Under Linux, the bluez Bluetooth stack is recommended. The package you need is usually called "bluetooth" (for Debian's apt-get). This package should already contain "bluez-utils", if not, install it as well.
On the project website you'll find a zip-archive containing useful scripts for Linux. For Bluetooth-connection, the script "btconnect" is recommended, followed by the NXT's name or its MAC adress. Make sure the user has access rights to devices called "rfcomm". Sometimes this means adding the current user to the group "dialout".
For Bluetooth under Linux the packages "dbns" and "dbns-x11" may also be required. The the command "bluetooth-applet --singleton" can be run from the console. After this, the script "btconnect" should work.

If you get the error message "Can't connect RFCOMM socket: Permission denied", try to remove the paired device inside the bluetooth-applet to force a new authorization.



*** USB:

For USB-connections under Windows, the official LEGO MINDSTORMS NXT Device Driver v1.02 or better has to be installed (also called "Fantom"). It can be downloaded here:
http://mindstorms.lego.com/support/updates/
Direct download link is this:
http://cache.lego.com/upload/contentTemplating/MindstormsUpdate/otherfiles/2057/upload1F7B2420-A5ED-44FF-9460-E262657029DC.zip

On Linux systems, the open-source library libusb has to be present. It can be retrieved using a packet manager, for example Debian's apt-get. If installing the package "libusb" alone does not work, also "libusb-dev" should be retrieved and installed.


To troubleshoot problems with opening connections to the NXT, the toolbox command 
>> DebugMode on
can be used before trying to open a handle.




3. QUICK START
--------------

This is a very short summary of the next section (Installation guide):

- Extract the archive anywhere you want, KEEPING DIRECTORY STRUCTURE
- The destination folder should contain a directory called  RWTHMindstormsNXT
- In MATLAB, go to "File", "Set Path...", "Add Folder..."
- Browse to the location you extracted to, and add the folder  RWTHMindstormsNXT
- Also add the folder  tools  (it is a sub-directory of  RWTHMindstormsNXT )
- Press Save to remember settings for future MATLAB sessions
- Run the command >>COM_MakeBTConfigFile<< from the MATLAB command window  or:
  edit a bluetooth-example ini-file from the toolbox folder to suit your
  configuration. You can skip this and the next step if you don't want to
  use Bluetooth.
- Establish a Bluetooth connection to your NXT using your adapter's driver software.
- Authenticate with the NXT (a passkey request should appear)
- You can now open Bluetooth or USB connections using the toolbox commands 
  >>COM_OpenNXT<< or >>COM_OpenNXTEx<<



4. INSTALLATION GUIDE
---------------------

The RWTH - Mindstorms NXT Toolbox is a collection of MATLAB-functions (so called m-files) and documentation / help files (mostly HTML). You have received these files in a compressed archive that just needs to be extracted to a directory of your choice. This folder can even be on an external hard disk, USB stick or network drive. However it is recommended to use a folder on a normal hard disk drive for performance reasons.
So just uncompress the archive and remember the folder you extracted it to.
Make sure that the internal sub-directory structure is kept! Also it is important that all files are located in a sub-folder called "RWTHMindstormsNXT" (without the ") for identification purposes.

Now inside MATLAB, go to the menu "File" and choose "Set Path...". Inside the new window, press "Add Folder...", and browse to the location where you extracted the files from the archive to in the previous step. Now select the folder "RWTHMindstormsNXT" and confirm. Repeat this step, and add the folder called "tools", which is a sub-folder of the previously added "RWTHMindstormsNXT"-directory. When done, press "Save" to remember these settings for future MATLAB sessions.

After adding these 2 folders to the MATLAB search path, the installation is complete. To verify the installation, you can type the following line

  info = ver('RWTHMindstormsNXT')

inside the MATLAB command window. Also the command >>COM_CloseNXT('all')<< should work and complete without an error.


*** Performance issues

On slower machines, CPU load during programs using the toolbox can be up to 100% (especially when constantly polling sensor or motor data via USB). To optimize the toolbox, a utility called >>OptimizeToolboxPerformance<< is provided, which can be called from the MATLAB command window. It will try to replace some frequently used  helper functions with binary versions from your MATLAB installation.
The tool will guide you through the process. 

Performance improvements up to a factor of 3 have been observed!



5. FIRST BLUETOOTH CONNECTION
-----------------------------

Before you can begin working with Bluetooth connections, you have to create a settings file that contains information about your Bluetooth adapter and serial port.
Either you can use the toolbox-command >>COM_MakeBTConfigFile<< inside the command window. A dialog window lets you enter the required parameters. The other way is to edit the example-files called "bluetooth-example-windows.ini" or "bluetooth-example-linux.ini", that are provided in the toolbox root folder.

You can leave the default values for the beginning, the only thing you will have to enter is the COM-Port. The Bluetooth SPP (serial port profile) maps a virtual COM-Port to your adapter. Find out which port this is (under Windows you can use the Device Manager) and enter it in the dialog window (example: COM4). The other parameters are explained in the documentation. Advanced users should refer to the first chapter "Bluetooth connections and serial handles" of the section "Functions - Overview".

If there is more than one additional Bluetooth COM-Port, this is most likely caused by the adapter's driver software. Most of the time it is the lowest available COM-Port (above the classic "real ports"). The only way to be sure is to try which ports are working. Sometimes there are certain ports that only work for sending OR receiving. The toolbox however needs a bidirectional port.)

Linux users should use the bluez Bluetooth stack. The serial port will then be called "/dev/rfcomm0" (without ") or similar. This is the parameter that has to be added instead of "COM3" for example. The sample ini-file for Linux does not contain all settings as they are not needed here.

Once the correct ini-file is created, it can be put inside the toolbox root path or anywhere inside the MATLAB search path for better convenience.

To establish the physical connection to your NXT, the script "btconnect" can be used. It is available on the project website. If you get the error message "Can't connect RFCOMM socket: Permission denied", try to remove the paired device inside the bluetooth-applet to force a new authorization. See also section 2 (System Requirements) of this document and follow the steps closely to ensure all needed packages are installed.


You can now try the demos (which require a correct configuration file called "bluetooth.ini") or start opening connections using the toolbox command COM_OpenNXT.

Note that before this works, you have to establish a physical connection to your NXT. Depending on your Bluetooth adapter's driver software, this can be different. Once successful, the NXT and driver software will prompt you for a passkey. The authentification is then complete, and the toolbox should work properly.



6. ACKNOWLEDGEMENTS
-------------------

See the text-file AUTHORS which comes with this distribution.



7. WEBSITES
-----------

*  Official project homepage
-> http://www.mindstorms.rwth-aachen.de


*  Institute of Imaging & Computer Vision, RWTH Aachen University
   (Project foundation, initial development and stable toolbox version maintenance)
-> http://www.lfb.rwth-aachen.de/en


*  RWTH Aachen University Student Project - MATLAB meets LEGO Mindstorms
-> http://www.lfb.rwth-aachen.de/mindstorms


*  Official LEGO MINDSTORMS NXT homepage
-> http://www.mindstorms.lego.com


*  The MathWorks, Inc. (MATLAB product updates and much more)
-> http://www.mathworks.com


*  RWTH Aachen University
-> http://www.rwth-aachen.de