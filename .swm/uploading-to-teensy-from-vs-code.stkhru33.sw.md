---
title: Uploading to Teensy from VS Code
---
# **Uploading a Sketch to Teensy 4.0 Using Visual Studio Code**

# *!!! May contain inaccuracies !!!*

This documentation guides you through the process of uploading a sketch to a Teensy 4.0 microcontroller using Visual Studio Code. By following these steps, you'll be able to leverage the power of Visual Studio Code for Arduino development and deploy your code onto the Teensy 4.0.

## **Prerequisites**

1. **Teensy 4.0 Board:** Ensure that you have a Teensy 4.0 microcontroller board.

2. **Visual Studio Code:** Install Visual Studio Code on your computer. You can download it from <https://code.visualstudio.com/>.

3. **Arduino Extension for Visual Studio Code:** Install the "Arduino" extension for Visual Studio Code. Open VS Code, go to the Extensions view (`Ctrl+Shift+X` or `Cmd+Shift+X`), search for "Arduino," and install the one provided by Microsoft.

## **Configuring Teensy in Visual Studio Code**

### **Step 1: Open Visual Studio Code**

Open Visual Studio Code on your computer.

### **Step 2: Configure Arduino Board Manager**

1. Open the command palette (`Ctrl+Shift+P` or `Cmd+Shift+P`).

2. Type and select "Arduino: Board Manager."

3. In the Board Manager, search for 'Teensy'

4. Select the latest version and click install

5. Close the Board Manager.

### **Step 3: Install Teensyduino Core**

1. Open the command palette again (`Ctrl+Shift+P` or `Cmd+Shift+P`).

2. Type and select "Arduino: Board Manager."

3. Search for "Teensy" and click "Install" to install the Teensyduino core.

### **Step 4: Select the Teensy Board and Port**

1. Open the command palette and select "Arduino: Change Board Type."

2. Choose "Teensy 4.0/4.1" from the list.

3. Select the port your Teensy is connected to by clicking on the bottom-right status bar (near the blue Arduino icon) and choose the appropriate port.

## **Uploading a Sketch to Teensy 4.0**

### **Step 1: Create or Open a Sketch**

Create a new Arduino sketch or open an existing one in Visual Studio Code.

### **Step 2: Verify and Compile the Sketch**

Click on the "Checkmark" icon in the top right (![](https://firebasestorage.googleapis.com/v0/b/swimmio.appspot.com/o/repositories%2FZ2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28%3D%2Ffcd3d970-d49a-42db-be52-c970b71cf337.png?alt=media&token=722f3148-1104-4943-8e82-22f1a93e3b90)) or use the command palette (`Ctrl+Alt+B` or `Cmd+Alt+B`) to verify and compile your Arduino sketch. Ensure that there are no compilation errors.

### **Step 3: Upload the Sketch**

Click on the "Arrow" icon in the top right (![](https://firebasestorage.googleapis.com/v0/b/swimmio.appspot.com/o/repositories%2FZ2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28%3D%2F7b690c2b-8e1a-49a5-a913-5ace73cb4320.png?alt=media&token=bffc8828-ab6b-49eb-a236-3b8c297c27d0)) or use the command palette (`Ctrl+Alt+U` or `Cmd+Alt+U`) to upload the sketch to your Teensy 4.0.

&nbsp;

## Serial Monitor

If not yet downloaded, download the Serial Monitor extension.

Once the teensy is connected, choose the COM port and the desired baud rate (default is 9600)

When pressing 'Start Monitoring', if all goes well, data should start coming in

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28=" repo-name="PARSEC_CODE"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
