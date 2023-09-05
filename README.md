# PartialPC_Processing
Workflow to combine point clouds from two Kinect Azure DK depth cameras and recreate missing data of a human subject. 

## Data collection
### Camera positioning
![image](https://github.com/EmmaRYoung/PartialPC_Processing/assets/67296859/65bb6980-62d6-4f99-8353-a88a9871bec0)

### Camera calibration
After setting up cameras at the right angle position above, collect a capture of "calibration objects". 
![image](https://github.com/EmmaRYoung/PartialPC_Processing/assets/67296859/e1769dc3-3e8e-463b-bde3-b161c367e6da)

Import these into [cloud compare](https://www.danielgm.net/cc/) to find a transformation matrix to transform the sub view into the master view. This is how "TransformSub.txt" is created. 

### Run PartialPCProcessing.m
Requirements
*.JSON file of the joint locations from the Kinect Azure
*First frame of the Kinect point cloud capture

The point cloud after the sub view is transformed to align with the master view

![image](https://github.com/EmmaRYoung/PartialPC_Processing/assets/67296859/4f41f249-4a5f-4a86-b958-cc963495300e)


The point cloud after the missing left side is recreated and the back is estimated

![image](https://github.com/EmmaRYoung/PartialPC_Processing/assets/67296859/e53f9bde-13d1-4e3e-a144-6cd3497676fb)
![image](https://github.com/EmmaRYoung/PartialPC_Processing/assets/67296859/22607544-9b9e-4ec3-9d62-db06497052c7)
