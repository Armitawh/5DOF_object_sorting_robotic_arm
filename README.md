# Machine Learning–Based 5-DOF Robotic Arm for Object Sorting

## 📌 Project Overview
This project implements a **machine learning–driven object sorting system** using a **5-Degree-of-Freedom (5-DOF) robotic arm**.  
The system combines **sensor data, machine learning algorithms, and robotic control logic** to autonomously classify objects and perform pick-and-place sorting tasks.

The project emphasizes **intelligent perception**, **decision-making**, and **robotic manipulation**, rather than low-level motor control alone.

---

## 🤖 Hardware Components
The system is built using the following hardware:

- **Raspberry Pi A** – Main processing and control unit  
- **RGB Sensor** – Color-based feature extraction for object classification  
- **Raspberry Pi Camera Module** – Visual perception and object identification  
- **Ultrasonic Sensor** – Distance measurement and object presence detection  
- **5-DOF Robotic Arm** – Object manipulation and sorting

Sensor data is processed on the Raspberry Pi and used as input for machine learning models that guide the robotic arm’s actions.

---

## 🧠 Machine Learning Models Used
Several supervised machine learning algorithms were implemented and compared:

- **K-Nearest Neighbors (KNN)**
- **Support Vector Machines (SVM)**
- **Decision Trees**
- **Random Forest**
- **Linear Regression**
- **Ridge Regression**

These models are used for:
- Object classification
- Feature-based decision making
- Performance comparison across multiple ML techniques

---

## 📊 Dataset Description
The dataset consists of **sensor-derived feature vectors** collected from the robotic system.

- Input features include RGB values, distance measurements, and extracted object features
- Output labels represent object categories or sorting decisions
- Data is split into training and validation sets for unbiased evaluation

---

## 📈 Evaluation Metrics

### Classification
- Accuracy
- Confusion matrix

### Regression
- Mean error
- Median error
- Standard deviation
- Error distribution analysis

These metrics assess both **classification reliability** and **continuous prediction performance**.

---

## 🛠️ Tools & Technologies
- **MATLAB**
- **Machine Learning Toolbox**
- **Raspberry Pi**
- **Computer Vision**
- **Supervised Machine Learning**
- **Robotic Manipulation Systems**

---

## 🧪 Experimental Results
- KNN and Random Forest models achieved high classification accuracy
- Regression models provided stable prediction behavior
- Results demonstrate the effectiveness of ML-driven decision-making for robotic sorting tasks

---

## 🔮 Future Work
- Full real-time deployment on embedded hardware
- Deep learning–based object recognition
- Sensor fusion for improved robustness
- Integration with AR-based monitoring and visualization
- Closed-loop control with adaptive learning



