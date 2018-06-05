# RhinoVR

RhinoVR is a Rhino 6 plug-in which uses the HTC Vive or Oculus Rift head-mounted displays to render Rhino viewports in virtual reality. This plug-in was created as an sample for developers to show how a VR plug-in can be developed for Rhino 6.

## Requirements:
* HTC Vive or Oculus Rift
* Rhino 6 (Service Release 6)

## Installation
Make sure you have at least Rhino 6 version 6.6.18152.23151. This version is currently available as a service release candidate. To enable the downloading of service release candidates, go to Tools -> Options... -> Updates and Statistics -> Update frequency: Service Release Candidate. Then click "Check Now...". Once you have the very latest version of Rhino 6 then proceed to download the RhinoVR RHI file from here (release section) and double click the file to install RhinoVR. The installer should indicate that it has installed RhinoVR for Rhino 6.

## Usage
Start Rhino and load the file you want to view in VR. Make sure a perspective viewport is selected and that it is set to the display mode you want to use. Type RhinoVR into the command line. Rhino tells you when it is ready and asks you to put the VR headset on.

* It is possible to navigate around in the Rhino scene using the VR controllers (Vive and Rift). Vive uses the touchpads for navigation and the Rift uses the analog sticks. The left controller controls translation (forward, backward, left, right) and the right controller controls horizontal rotation (turning left/right) and up/down movement.
* Objects can be selected by pressing the touchpad button (Vive) or pressing the analog stick (Rift).
* The Rhino Move-command can be initiated by pressing the Application Menu button (Vive) or the B-button (Rift). Remember, the move command works in steps: Select objects. Enter. Pick reference point. Pick new point.
* A Rhino Enter key press can be mimicked using the VR controller by pressing the trigger button (Vive) or the A-button (Rift).
* A Rhino Esc key press can be mimicked by squeezing the grip buttons (Vive) or by pressing the primary trigger button (Rift).

## Licenses
RhinoVR uses the open source library OpenVR. See license details in the OpenVR subfolder.
