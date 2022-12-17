# enph353
Team 11

An autonomous parking agent that can fully navigate a simulated enviroment whilst detecting and reporting liscense plates, a project created for ENPH353

| Source File  |Description |
| ------------- | ------------- |
| PID.py | Used for autonomous robot control, in order to navigate it around the competition surface |
| SIFT.py  | Used to collect liscense plates while navigating around the surface  |
| image_processing.py  | Used to process the data afterwards, performing SIFT, perspective transforms, and character detection through CNNs |
| location_cnn.py  | Used to train the location CNN, which is located in the /location_cnn folder  |
| plate_cnn.py | Used to train the location CNN, which is located in the /plate_cnn folder |
| score_interface.py | Used to start and stop the timer used in the competition, allowing for a execution under the time limit  |
| plate_generator.py | Used for generating raw data for the liscense plates used in the competition|


All additional helper functions in helper nodes were used to ensure proper code decomposition and reuse
