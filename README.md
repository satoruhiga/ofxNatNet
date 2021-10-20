# ofxNatNet

The original ofxNatNet addon by [satoruhiga](https://github.com/satoruhiga/ofxNatNet).
Support NatNet SDK up to 3.1. (WIP)

## Setting on Motive
- See [Motive document](https://v22.wiki.optitrack.com/index.php?title=Data_Streaming) to enable data streaming.
- If you use NatNet SDK 3.0 or higher, acquiring rigid body markers got difficult a littile bit.
    - On Motive Streaming pane, enable `Asset Markers` as well as `Rigid Bodies`.
    - Then you can see rigid body markers in the same manner with NatNet 2.x.
    - However, if you make new rigid bodies in Motive during app running, you have to call `ofxNatNet::sendRequestDescription()` to re-create conversion table between streaming id and name, because asset markers are only associated with name.


## Reference Implementation
- https://github.com/mje-nz/natnet-sdk-mirror/blob/master/Samples/PythonClient/NatNetClient.py

## Tested Environment
- oF 0.11.2 macos
- oF 0.11.2 Windows 10