# Autonomous Machine Learning Parking Agent
An autonomous parking agent that can fully navigate a simulated enviroment whilst detecting and reporting liscense plates, a project created for ENPH353

| Source File  |Description |
| ------------- | ------------- |
| PID.py | Used for autonomous robot control, in order to navigate it around the competition surface |
| Image_Detection.py  | Used to collect liscense plates while navigating around the surface  |
| SIFT.py  | Used to process the data afterwards, performing SIFT, perspective transforms, and character detection through CNNs |
| location_cnn.py  | Used to train the location CNN, which is located in the /location_cnn folder  |
| plate_cnn.py | Used to train the location CNN, which is located in the /plate_cnn folder |
| score_interface.py | Used to start and stop the timer used in the competition, allowing for a execution under the time limit  |
| plate_generator.py | Used for generating raw data for the liscense plates used in the competition|


All additional helper functions in helper nodes were used to ensure proper code decomposition and reuse

![image](https://user-images.githubusercontent.com/74876670/211437300-307e65d0-2d4a-43e8-ae21-3034172d8305.png)

<sup> This repository was created for ENPH353, a competition based course that involves engineering project planning, execution and reporting. The course involves carrying out an open-ended Engineering project to meet specific performance metrics on an industry relevant topic selected by instructors. Reporting on progress is both oral and written. </sub>

Additional files of the robot moving and files related to the robot are avaliable on request

