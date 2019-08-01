# ISI HES Probe ROS Driver

Converts from Labview string topic to sensible custom message types.
While the LabView for ROS package should be able to publish custom message types the documentation is not good and so we were unable to impliment it. Instead, the strings are passed to this conversion node that takes the strings and puts them into custom message types

## 1. Nodes

## 1.1 Subscribed topics

/isi/wavenumber (std_msgs/string)

- Wave number of spectra (y axis). Message is comma seperated string containing a 1x1000 array of doubles.

/isi/intensity (std_msgs/string)

- Intensity of wave number in spectra (x axis). Message is comma seperated string containing a 1x1000 array of doubles.

/isi/ml (std_msgs/string)

- Machine learning result from analysing spectra. Message is a comma seperated string containing a 2x5 arrray. First row is the top 5 of chemical names as strings. Second row is the similarity rating of the top 5 chemicals.

/isi/header (std_msgs/string)

- Information on calibration and exposure from HES probe (currently not used but may be implimented in future). Message is a string.

## 1.2 Published topics

isi/hes_spectra (isi_hes_msgs/spectra)

- Spectral measurement. Message contains wavenumber and intensity (x,y) for spectra.

isi/hes_ml_result (isi_hes_msgs/ml)

- Machine learning result from analysing spectra. Message contains a list the top 5 chemial names as strings and a second list of the similarity rating of each of the top 5 chemicals.

## 1.3 Custom mesage type

isi_hes_msgs/spectra

- See isi_hes_msgs README for details on custom message types

isi_hes_msgs/ml)

- See isi_hes_msgs README for details on custom message types
