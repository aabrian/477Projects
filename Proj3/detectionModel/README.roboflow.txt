
Project3Group3 - v2 2023-04-23 10:29pm
==============================

This dataset was exported via roboflow.com on May 1, 2023 at 7:23 PM GMT

Roboflow is an end-to-end computer vision platform that helps you
* collaborate with your team on computer vision projects
* collect & organize images
* understand and search unstructured image data
* annotate, and create datasets
* export, train, and deploy computer vision models
* use active learning to improve your dataset over time

For state of the art Computer Vision training notebooks you can use with this dataset,
visit https://github.com/roboflow/notebooks

To find over 100k other datasets and pre-trained models, visit https://universe.roboflow.com

The dataset includes 306 images.
Legos-robots-obstacles are annotated in YOLO v5 PyTorch format.

The following pre-processing was applied to each image:
* Auto-orientation of pixel data (with EXIF-orientation stripping)

The following augmentation was applied to create 3 versions of each source image:
* 50% probability of horizontal flip
* Randomly crop between 0 and 10 percent of the image
* Random rotation of between -15 and +15 degrees
* Random shear of between -10° to +10° horizontally and -5° to +5° vertically
* Random brigthness adjustment of between -20 and +20 percent
* Random exposure adjustment of between -8 and +8 percent
* Random Gaussian blur of between 0 and 3 pixels
* Salt and pepper noise was applied to 3 percent of pixels


