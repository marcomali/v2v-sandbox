# v2v-sandbox

ns3 modules to build a simple V2V application using SUMO (v-1.2.0) and ns-3 (v-3.28).

It has been tested with SUMO v1.2.0 and ns3 v3.28 on Ubuntu 18.04.
Back compatibility **is not** ensured with new versions of TraCI.

To build the project:
* Install SUMO following the guide at [https://sumo.dlr.de/wiki/Downloads](https://sumo.dlr.de/wiki/Downloads)
    * You can use 
    	`sudo add-apt-repository ppa:sumo/stable`  
    	`sudo apt update`  
    	`sudo apt install sumo sumo-tools sumo-doc`  
    * Be careful: in the future the previous commands will install updated version of SUMO which are not ensured to work with this scripts (that are tested wit **v-1.2.0**)
* Clone this repository in your pc.
    
* Configure waf to build the new modules with "./waf configure --build-profile=optimized --enable-examples --enable-tests --enable-sudo" (add here what you want to enable) - Using the optimized profile allows to speed up the simulation time
* Build ns3 with "./waf build"

**Important**
The final project path-tree should be like:

    automotive/
               doc/
               examples/
                        sumo-files/
                                  img/
               helper/
               model/
                    asn1/
               test/
    traci/
          doc/
          examples/
          model/
    traci-applications/
                       examples/
                       helper/
                       model/

automotive/ contains all the application related files. Inside sumo-files you can find the SUMO map, trace and even images.
traci/ and traci-applications/ contains all the logic to link ns-3 and SUMO.
It is important to keep the folder tree that way, otherwise the simulation won't work properly.


**Simple V2V example**

To run the program:

`./waf --run "v2v-cv2x-sandbox"` or
`./waf --run "v2v-80211p-sandbox"`


*  Nodes are created in the ns3 simulation as vehicle enters the SUMO simulation
*  A full cv2x or 802.11p stack is implemented at lower layers

In this example, every vehicle that enters the scenario will start sending CAM (in plain text) with freq 10 hz. The vehicles around will only receive the message and increment a counter. To switch to ASN.1 format, use the command --asn=true.
The mobility trace is managed by the file automotive/example/sumo-files/cars.rou.xml -> please note that the very first line of this file are used to determine the number of UE to be generated in the simulation.
The application managing the CAM dissemination (automotive/model/v2c-CAM-sender.cc) comes with two functions to encode and send plain-tex or ASN.1 DENMs. Moreover, the application is written to support DENM decoding.

**List of most important commands**
* --realtime				           [bool] decide to run the simulation using the realtime scheduler or not
* --sim-time                   [double] simulation time
* --sumo-gui                   [bool] decide to show sumo-gui or not
* --sumo-updates 			         [double] frequency of SUMO updates
* --send-cam 				           [bool] enable vehicles to send CAM
* --asn                        [bool] if true, CAMs and DENMs are encoded and decoded using ASN.1 
* --cam-intertime              [double] CAM dissemination intertime

**IMPORTANT**
Sometimes it may happen that in build phase you have some "Warning threated as error". To remove that, configure the project using:

`CXXFLAGS="-Wall" ./waf configure --build-profile=optimized --enable-examples --enable-tests --enable-sudo`

and then build again
