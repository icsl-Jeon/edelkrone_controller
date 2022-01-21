# Edelkrone controller 


### Installation (tested on anaconda, python 3.9, Windows 10)
#### 1, pyzed 

Assuming ZED SDK is installed,
```shell
conda activate <your_env>
cd <path/to/ZED SDK>
python .\get_python_api.py
```

Launch python in new terminal. 
```
import pyzed
import pyzed.sl
```


#### 2. opencv 

````shell
pip install opencv-python
````

### Extrinsic calibration 
##### Step 1 
Turn on webapp and prepare to move the motors using GUI.
Then, launch 
```shell
python extrinsic_edelkron_manual
```

##### Step 2 
Launch the [matlab file](matlab/calibration.m). 


