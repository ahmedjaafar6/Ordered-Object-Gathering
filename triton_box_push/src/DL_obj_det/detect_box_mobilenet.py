#!/usr/bin/env python3

from transformers import AutoImageProcessor, MobileNetV2ForImageClassification
import torch

image_processor = AutoImageProcessor.from_pretrained("google/mobilenet_v2_1.0_224")

model = MobileNetV2ForImageClassification.from_pretrained("google/mobilenet_v2_1.0_224")
