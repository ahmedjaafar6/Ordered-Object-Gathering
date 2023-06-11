#!/usr/bin/env python3

import time
import torch

if torch.cuda.is_available():
    print("cuda is available")
else:
    print("cude is NOT available")
    
    
model_path = "./effdet_torch/efficientdet-d0.pth"

"""
model = EfficientDetBackbone(compound_coef=compound_coef, num_classes=len(obj_list),
                             ratios=anchor_ratios, scales=anchor_scales)
model.load_state_dict(torch.load(model_path, map_location='cpu'))
model.requires_grad_(False)
model.eval()
"""
