## warning:
There is a [threading issue](https://github.com/MetalWorkerTools/grbl-L-Mega/issues/5#issue-1016791195). Read the [issue](https://github.com/MetalWorkerTools/grbl-L-Mega/issues/5#issue-1016791195) for more information.
***
New features in the current [CNCL](https://www.microsoft.com/store/apps/9P42TB5T697H) release requires some GRBL compiler options changes. These compiler options changes are implemented in this GRBL fork to make it easy to compile and flash the modified GRBL to an Arduino for running on a lathe. The [Compiler Options](https://github.com/HuubBuis/grbl-L-Mega/wiki/Changed-Compiler-options) are described in the [GRBL-L-Mega Wiki](https://github.com/HuubBuis/grbl-L-Mega/wiki). For all other information read the [GRBL Wiki](https://github.com/gnea/grbl/wiki)  
I have added spindle sync threading G33 based on [fschill grbl version](https://github.com/fschill/grbl-Mega/tree/spindle_sync) to this GRBL-L-Mega version and the version for the Arduino Uno [GRBL-L](https://github.com/HuubBuis/grbl-L). Read the [threading setup Wiki](https://github.com/HuubBuis/grbl-L-Mega/wiki/Threading-setup-and-use) for an explanation of the threading setup.  

Ramps 1.4 - 1.6 users use the Ramps branch.
