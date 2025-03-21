# Event-based Camera Calibration Using Vanishing Points

## 📌 Overview
This repository provides a MATLAB implementation for event-based camera calibration using vanishing points. The method leverages event data processing techniques to efficiently extract vanishing points and optimize camera parameters.

## 🚀 Features
- **Automated Vanishing Point Detection** – Robustly detects vanishing points from event data.
- **MATLAB-Based Implementation** – Easily integrates with MATLAB’s powerful toolboxes.
- **Modular Pipeline** – Scripts can be executed independently based on specific calibration needs.
- **Visualization Support** – Generates detailed visual outputs for validation.

## 🛠 Installation & Setup
Follow these steps to set up the repository on your system:

1. **Clone the repository:**
   ```sh
   git clone https://github.com/SezginDulkadir/Event-based-Camera-Calibration-Using-Vanishing-Points.git
   cd Event-based-Camera-Calibration-Using-Vanishing-Points
   ```

2. **Add the repository to the MATLAB path:**
   ```matlab
   addpath(genpath('Event-based-Camera-Calibration-Using-Vanishing-Points'));
   ```

3. **Ensure the following MATLAB toolboxes are installed:**
   - Image Processing Toolbox
   - Computer Vision Toolbox
   - Camera Calibration Toolbox

## 📌 Usage
This repository contains several key scripts, each performing a specific function:

| Script | Description |
|---------|-------------|
| `DBSCAN_Dense_Event_Detection.m` | Detects dense event clusters from event data. |
| `RANSAC_Cube_Projection_Optimization.m` | Optimizes 3D cube projection using RANSAC. |
| `Geometric_Line_Intersection_Detection.m` | Identifies geometric line intersections and extracts vanishing points. |
| `Camera_Calibration_Using_Vanishing_Points.m` | Performs camera calibration using detected vanishing points. |

### Running a Script
To execute a specific script, use:
```matlab
run('script_name.m');
```
Refer to inline comments within each script for details on parameters and customization.

## 🐄 Dataset
A dataset is available for testing and benchmarking the calibration method. Download it from the following link:

📞 [https://drive.google.com/drive/folders/1P11lBoNIk-n40t057sqY-o6jH4SK_ROP?usp=drive_link](#) 

To use your own dataset:
1. Place the files inside the `dataset/` folder.
2. Update the configuration settings accordingly.

## 🎯 Results
The repository includes example visualizations of vanishing point detections and calibration outputs. Sample results are available in the `results/` folder.

### Calibration Pipeline
A high-level overview of the event-based camera calibration process:

![Calibration Pipeline](images/calibration_pipeline.png)

### Vanishing Points Detection
An example visualization of detected vanishing points:

![Vanishing Points](images/vanishing_points_example.png)

### Example Output
Here is an example output of the calibration process:

![Example Output](images/example_output.png)

## 🤝 My Contributions
In this project, I contributed to the following:

- **Dense Event Data Detection:** Implemented DBSCAN algorithm to detect dense clusters in event data.
- **Optimization of 3D Cube Projection:** Used RANSAC to optimize 3D cube projections and enhance calibration accuracy.
- **Geometric Line Intersection Detection:** Developed methods to extract vanishing points using geometric line intersections.
- **Visualization Tools Development:** Created visualization tools for validating detection and calibration results.

## 🤝 Contributing
Contributions are welcome! To contribute:
1. Fork this repository.
2. Create a new branch (`feature-branch`).
3. Commit your changes.
4. Open a pull request (PR).

For major changes, please open an issue first to discuss your ideas.

## 📝 License
This project is licensed under the MIT License.

## 💎 Contact
For questions or support, feel free to reach out via GitHub Issues.

sezgindulkadir@aybu.edu.tr

