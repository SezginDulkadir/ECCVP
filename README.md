# Event-based Camera Calibration Using Vanishing Points

## Overview
This repository provides an implementation of event-based camera calibration using vanishing points. The method utilizes event-based data processing techniques to extract vanishing points and optimize the camera parameters efficiently.

## Features
- **DBSCAN Clustering:** Detects dense event clusters.
- **RANSAC Optimization:** Projects 3D cubes and refines calibration.
- **Geometric Line Intersection:** Identifies vanishing points.
- **Visualization Tools:** Displays results for verification.

## Installation
To use this repository, clone it and install the required dependencies:

```bash
git clone https://github.com/SezginDulkadir/Event-based-Camera-Calibration-Using-Vanishing-Points.git
cd Event-based-Camera-Calibration-Using-Vanishing-Points
pip install -r requirements.txt
```

## Usage
Run the main script to perform event-based calibration:

```bash
python main.py --input dataset/sample_data.npy
```

For more options, use:

```bash
python main.py --help
```

## Dataset
A dataset is available for testing and benchmarking the method. You can download it from the following link:

[https://drive.google.com/drive/folders/1P11lBoNIk-n40t057sqY-o6jH4SK_ROP?usp=sharing](#)  
(*Replace with actual dataset link*)

To use your own dataset, place it inside the `dataset/` folder and update the configuration accordingly.

## Results
The repository includes example visualizations of vanishing point detections and calibration outputs. See the `results/` folder for sample outputs.

## License
This project is licensed under the MIT License.

## Contact
For any inquiries, feel free to reach out via GitHub Issues.




