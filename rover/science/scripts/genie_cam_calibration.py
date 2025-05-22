import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

'''
What this script does (image-based spectral calibration):
1. Loads multiple types of calibration and sample images.
2. Averages them to remove noise using bias, dark, and flat corrections.
3. Computes reflectance of a sample.
4. Optionally, analyze a region of interest (ROI).
5. Plots reflectance as a function of wavelength.

How to use:
1. Set the paths to your folders containing bias, dark, flat, calibration target, and sample images.
2. Set the wavelengths corresponding to the calibration target.
3. Optionally, set the ROI as (x1, y1, x2, y2) to extract a region of interest.
4. Run the script to perform calibration and plot the spectral reflectance curve.
'''

def load_images(folder):
    # load all images in a folder as greyscale numpy arrays
    images = []
    for fname in sorted(os.listdir(folder)): # sorting alphabetically and looping through each file
        if fname.lower().endswith(('.png', '.jpg', '.jpeg', '.tif', '.tiff', '.bmp')):
            img = cv2.imread(os.path.join(folder, fname), cv2.IMREAD_GRAYSCALE) # load and convert to greyscale
            images.append(img.astype(np.float32)) # convert to float32
    return np.stack(images, axis=0)  # shape: (num_images, H, W)

def mean_image(images): 
    # compute the mean image from a stack of images
    return np.mean(images, axis=0)

def calibrate_images(
    bias_folder, dark_folder, flat_folder, calibration_folder, sample_folder, wavelengths, roi=None
):
    # loading images from folders
    bias_imgs = load_images(bias_folder)
    dark_imgs = load_images(dark_folder)
    flat_imgs = load_images(flat_folder)
    cal_imgs = load_images(calibration_folder)
    sample_imgs = load_images(sample_folder)

    # computing the mean frames
    bias = mean_image(bias_imgs)
    dark = mean_image(dark_imgs) - bias
    flat = mean_image(flat_imgs) - dark
    flat_norm = flat / np.mean(flat)

    # correct calibration and sample images
    cal_corrected = (cal_imgs - dark) / flat_norm
    sample_corrected = (sample_imgs - dark) / flat_norm

    # computing normalized reflectance (using reflectance transformation formula)
    reflectance = sample_corrected / cal_corrected  # shape: (num_filters, H, W)

    # extract ROI (optional) and compute mean reflectance per wavelength
    if roi is not None:
        x1, y1, x2, y2 = roi  # ROI as (x1, y1, x2, y2)
        reflectance_roi = reflectance[:, y1:y2, x1:x2]
    else:
        reflectance_roi = reflectance

    mean_reflectance = np.mean(reflectance_roi, axis=(1, 2)) # shape: (num_filters,)
    # plotting spectral reflectance curve
    plt.figure(figsize=(8, 5))
    plt.plot(wavelengths, mean_reflectance, marker='o')
    plt.xlabel('Wavelength (nm)')
    plt.ylabel('Reflectance')
    plt.title('Spectral Reflectance Curve')
    plt.grid(True)
    plt.show()

    return mean_reflectance

if __name__ == "__main__":
    # SET FOLDERS AND WAVELENGTHS HERE!!!   
    bias_folder = "data/bias"
    dark_folder = "data/dark"
    flat_folder = "data/flat"
    calibration_folder = "data/calibration_target"
    sample_folder = "data/sample"
    wavelengths = [450, 550, 650, 750, 850, 950, 1050, 1150, 1250, 1350, 1450, 1550]  # Example

    # optionally, set ROI as (x1, y1, x2, y2)
    roi = (100, 100, 400, 400)

    mean_reflectance = calibrate_images(
        bias_folder, dark_folder, flat_folder, calibration_folder, sample_folder, wavelengths, roi
    )