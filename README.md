Gait-Sensing Wearable System: Hardware + Web Interface
This repository contains the full implementation of a multimodal wearable gait-sensing system developed for the DESE71003 – Sensing and Internet of Things module at Imperial College London.
It includes:
ESP32-S3 firmware for MMG, EMG and IMU acquisition
BLE streaming pipeline for real-time transmission
Web-based interface for visualisation and machine-learning inference
Offline data-processing and model-training scripts
The system recognises walking, jumping, and stair ascent using lightweight logistic regression deployed directly in the browser.
