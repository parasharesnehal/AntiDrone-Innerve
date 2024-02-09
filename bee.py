import os
import cv2
import sys
import time
import numpy as np
import datetime
from math import sqrt

import ardrone_library as al  # Renamed libardrone to ardrone_library
from object_tracker import ObjectTracker  # Renamed tracker.py to object_tracker.py
from mvnc import mvncapi as mvnc
import random

random_color = lambda: random.randint(0,255)

box_color00 = (random_color(), random_color(), random_color())
box_color01 = (random_color(), random_color(), random_color())
box_color10 = (random_color(), random_color(), random_color())
box_color11 = (random_color(), random_color(), random_color())
global_box_color = (random_color(), random_color(), random_color())

input_dimension = (300, 300)

CLASSES = ('background',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor',
           'heli')

SPLIT = False
CLUSTERING = True

width = 640
height = 320

command_time = False
min_confidence_percent = 8

def euclidean_distance(p1, p2):
    return sqrt((p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

def cluster_centers(centers, d):
    clustered_centers = []
    d_squared = d * d
    n = len(centers)
    used = [False] * n
    for i in range(n):
        if not used[i]:
            cluster_center_count = 1
            center = [centers[i][0], centers[i][1], centers[i][2], centers[i][3], centers[i][4]]
            used[i] = True
            for j in range(i+1, n):
                if euclidean_distance(centers[i], centers[j]) < d_squared:
                    center[0] = max(center[0], centers[j][0])
                    center[1] += centers[j][1]
                    center[2] += centers[j][2]
                    cluster_center_count += 1
                    used[j] = True
            center[1] /= cluster_center_count
            center[2] /= cluster_center_count
            clustered_centers.append((center[0], center[1], center[2], center[3], center[4]))
    return clustered_centers

# Modify function names and comments
def perform_inference(image_to_analyze, network_graph, width, height, w_base, h_base, box_color=(255, 128, 0)):
    resized_image = preprocess_image(image_to_analyze)
    network_graph.LoadTensor(resized_image.astype(np.float16), None)
    output, _ = network_graph.GetResult()

    num_detections = int(output[0])
    detected_boxes = []

    for box_index in range(num_detections):
        base_index = 7 + box_index * 7
        if any(not np.isfinite(output[base_index + i]) for i in range(7)):
            continue
        if output[base_index + 1] != 15:  # Filtering for 'person' class
            continue
        if int(output[base_index + 2] * 100) <= min_confidence_percent:
            continue

        x1 = max(0, int(output[base_index + 3] * width)) + w_base
        y1 = max(0, int(output[base_index + 4] * height)) + h_base
        x2 = min(width, int(output[base_index + 5] * width)) + w_base
        y2 = min(height, int(output[base_index + 6] * height)) + h_base

        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        box_width = x2 - x1
        box_height = y2 - y1

        detected_boxes.append(((output[base_index + 2] * 100), center_x, center_y, box_width, box_height))

        overlay_on_image(image_to_analyze, output[base_index:base_index + 7], min_confidence_percent, box_color)

    return detected_boxes

# Modify function names and comments
def overlay_on_image(display_image, object_info, min_confidence_percent, box_color):
    source_image_width = display_image.shape[1]
    source_image_height = display_image.shape[0]

    base_index = 0
    class_id = object_info[base_index + 1]
    confidence_percentage = int(object_info[base_index + 2] * 100)
    if confidence_percentage <= min_confidence_percent:
        return

    label_text = CLASSES[int(class_id)] + " (" + str(confidence_percentage) + "%)"
    box_left = int(object_info[base_index + 3] * source_image_width)
    box_top = int(object_info[base_index + 4] * source_image_height)
    box_right = int(object_info[base_index + 5] * source_image_width)
    box_bottom = int(object_info[base_index + 6] * source_image_height)

    box_thickness = 2
    cv2.rectangle(display_image, (box_left, box_top), (box_right, box_bottom), box_color, box_thickness)

    label_background_color = (125, 175, 75)
    label_text_color = (255, 255, 255)
    label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    label_left = box_left
    label_top = box_top - label_size[1]
    if label_top < 1:
        label_top = 1
    label_right = label_left + label_size[0]
    label_bottom = label_top + label_size[1]
    cv2.rectangle(display_image, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1),
                  label_background_color, -1)

    cv2.putText(display_image, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_text_color, 1)

# Modify function names and comments
def preprocess_image(src):
    NETWORK_WIDTH = 300
    NETWORK_HEIGHT = 300
    img = cv2.resize(src, (NETWORK_WIDTH, NETWORK_HEIGHT))
    img = img - 127.5
    img = img * 0.007843
    return img

# Modify variable names and comments
def update_drone_commands(center, tracker, tracker_width, tracker_height, drone):
    x_raw_distance
