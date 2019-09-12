# ISI HES Probe ROS Driver

## Description
Communicates with LabView program controlling the ISI HES Spectrometer. This node Converts from Labview string topics to sensible custom message types.
While the LabView for ROS package should be able to publish custom message types the documentation is not good and so we were unable to impliment it. Instead, the strings are passed to this conversion node that takes the strings and puts them into custom message types.

## Control state machine
The LabView program uses a state machine with the below states. This state is set using the "isi_hes/command" topic however empty services have been created for simplifying the control of each state so this topic should not be used directly. 

1 - Wait (Service: /isi_hes/wait)
This is an idle state that the state machine is returned to after completing another state. The return to this state is not done automatically so if the program is in the state 'range' then it will continue to take range measurements untill it is returned to the 'wait' state. However, the services for each of the states will automatically send the 'wait' command after running each of the other state calls to bring the program back to the wait state. 

2 - Range (Service: /isi_hes/get_range)
The probe must be at 1m from the sample for a useful measurement to be taken so a range measurement is done using stereo camera. The laser is turned off for this range measurement. 

3 - Check (Service: /isi_hes/check)
The laser is turned on and the image from the stereo camera is checked to see if the laser spot is visible and in the expected positon.

4 - Aquire (Service: /isi_hes/acquire)
If the range and spot check are sucessful then a raman measurement is done. The data from this aquisition is captured by this node as string msgs which are converted to relivent custom msgs: isi_hes_msgs/spectra and isi_hes_msgs/ml. 

5 - Stop (Service: /isi_hes/stop)
Stop the currently running state

6 - Laser On (Only used internall)

7 - Laser Off (Only used internally)

## Run

(1) - Run processing node
```bash
rosrun isi_hes_driver-ros process_labview_msgs
```

(2) - Move spectrometer aproximately 1m from sample.

(3) - Measure range to sample using service call
```bash
rosservice call /isi_hes/get_range
```

(4) - Wait for "Range response: [...]" message in console running process_labview_msgs node. If message is "Range response: out_of_range" check the position of the probe from the sample and re-run (3). Otherwise if message is "Range reponse: in_range" continue to (5).

(5) - Check laser spot using service call
```bash
rosservice call /isi_hes/check
```

(6) - Wait for "Range response: [...]" message in console running process_labview_msgs node. If message is "check_failed" make sure laser was turned on correctly and there are no obstructions between the laser and sample then re-run (5). Otherwise if message is "Range response: check_passed" continue to (7).

(7) - Run spectrometer measurement with service call
```bash
rosservice call /isi_hes/acquire
```

(8) - Wait for measurement to complete. The result will be published to string messages which will be handelled by process_labview_msgs and convert them to the two topics: isi/hes_spectra and isi/hes_ml_result. You should be subscibed to these messages to see the result. (You can check this by echo-ing the topic to the console)
```bash
rostopic echo /isi/hes_spectra
rostopic echo /isi/hes_ml_result
```


## 1. Nodes

process_labview_msgs
This handels service calls and passes messages between ros and LabView program. 

## 1.1 Subscribed topics

/isi/wavenumber_raw (std_msgs/string)

- Wave number of spectra (y axis). Message is comma seperated string containing a 1x1000 array of doubles.

/isi/intensity_raw (std_msgs/string)

- Intensity of wave number in spectra (x axis). Message is comma seperated string containing a 1x1000 array of doubles.

/isi/ml_raw (std_msgs/string)

- Machine learning result from analysing spectra. Message is a comma seperated string containing a 2x5 arrray. First row is the top 5 of chemical names as strings. Second row is the similarity rating of the top 5 chemicals.

/isi/header_raw (std_msgs/string)

- Information on calibration and exposure from HES probe (currently not used but may be implimented in future). Message is a string.

## 1.2 Published topics

isi/hes_spectra (isi_hes_msgs/spectra)

- Spectral measurement. Message contains wavenumber and intensity (x,y) for spectra. Along position of sample and pose when measurement was taken. 

isi/hes_ml_result (isi_hes_msgs/ml)

- Machine learning result from analysing spectra. Message contains a list the top 5 chemial names as strings and a second list of the similarity rating of each of the top 5 chemicals. Along position of sample and pose when measurement was taken. 

## 1.3 Custom mesage type

isi_hes_msgs/spectra

- See isi_hes_msgs README for details on custom message types

isi_hes_msgs/ml

- See isi_hes_msgs README for details on custom message types

## 1.4 Services

/isi_hes/wait (type:EmptyService)

- Set spectrometer to idle state waiting for commands. Mostly used in testing the communciations. LabView program uses state machine and returns to 'wait' state after running commands. This forces the program back to the wait state. 

/isi_hes/get_range (type:EmptyService)

- Trigger the range checking in the spectrometer LabView program. The distance check response is returned on the topic "/isi_hes/distance_raw" with string "in_range" or "out_of_range". 

/isi_hes/check (type:EmptyService)

- Trigger the spot check in the spectrometer LabView program. This check the existance of the laser spot and that it is in the expected location. The spot check response is returned on the topic "/isi_hes/distance_raw" with string "check_failed" or "check_passed". 

/isi_hes/acquire (type:EmtpyService)

- Trigger aquisition in the spectronmeter LabView program. This will only run if the distance and check states have returned sucessfully. The response from the aquisition is returned on the topics ending with the suffix '_raw'. /isi_hes/wavenumber_raw, /isi_hes/intensity_raw, /isi_hes/ml_raw, /isi_hes/header_raw, /isi_hes/distance_raw. 

/isi_hes/stop (type:EmptyService)

- Trigger a stop in the spectrometer LabView program. DO NOT USE AS EMERGENCY STOP THIS IS NOT SAFE. Will stop the currently running state and return to 'wait' state. 
