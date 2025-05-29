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

relative_spectral_sensitivity = {
    440: 0.78,
    500: 0.97,
    530: 1.0,
    570: 0.96,
    610: 0.9,
    670: 0.76,
    740: 0.58,
    780: 0.44,
    840: 0.31,
    900: 0.18,
    950: 0.1,
    1000: 0.05
}

wavelengths = [440, 500, 530, 570, 610, 670, 740, 780, 840, 900, 950, 1000]

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

def get_reflectance(filepath: str) -> dict:
    with open(filepath, "r") as f:
        lines = f.read().split("\n")
        d = {
            440: 0.0,
            500: 0.0,
            530: 0.0,
            570: 0.0,
            610: 0.0,
            670: 0.0,
            740: 0.0,
            780: 0.0,
            840: 0.0,
            900: 0.0,
            950: 0.0,
            1000: 0.0
        }
        for i in wavelengths:
            d[i] = float(lines[i - 349])
        return d

def calibrate_images(
    bias_folder, dark_folder, flat_folder, calibration_folder, sample_folder, wavelengths, roi=None, chalk_roi=None, hematite_roi=None, magnetite_roi=None, serpentine_roi=None, chlorite_roi=None
):
    # loading images from folders
    bias_imgs = load_images(bias_folder)
    dark_imgs = load_images(dark_folder)
    flat_imgs = load_images(flat_folder)
    cal_imgs = load_images(calibration_folder)
    sample_imgs = load_images(sample_folder)

    # computing the mean frames
    bias = mean_image(bias_imgs)
    dark_corr = mean_image(dark_imgs) - bias
    flat_corr = mean_image(flat_imgs) - dark_corr
    flat_norm = flat_corr / np.mean(flat_corr)

    # After computing dark_corr and flat_norm
    output_dir = "../genie_calibration_data"
    os.makedirs(output_dir, exist_ok=True)

    # Save the calibration matrices
    np.save(os.path.join(output_dir, "dark_correction.npy"), dark_corr)
    np.save(os.path.join(output_dir, "flat_normalization.npy"), flat_norm)

    # correct calibration and sample images
    cal_corrected = (cal_imgs - dark_corr) / flat_norm
    sample_corrected = (sample_imgs - dark_corr) / flat_norm

    # computing normalized reflectance (using reflectance transformation formula)
    # reflectance = sample_corrected / cal_corrected  # shape: (num_filters, H, W)

    for i in range(len(wavelengths)):
        cal_corrected[i] /= relative_spectral_sensitivity[wavelengths[i]]
        sample_corrected[i] /= relative_spectral_sensitivity[wavelengths[i]]
    
    with open("../genie_calibration_data/chalk - pigments checker acrylic - GorgiasUV.txt", "r") as fca:
        chalk_reflectance = {
            440: 0.0,
            500: 0.0,
            530: 0.0,
            570: 0.0,
            610: 0.0,
            670: 0.0,
            740: 0.0,
            780: 0.0,
            840: 0.0,
            900: 0.0,
            950: 0.0,
            1000: 0.0
        }
        lines = fca.read().split("\n")
        i = 0
        min_diff = 1.0
        val = 0
        for line in lines[1:-2]:
            l = line.split(" ")
            diff = abs(float(l[0]) - wavelengths[i])
            if diff < 1:
                if diff < min_diff:
                    min_diff = diff
                    val = float(l[1]) / 100
                else:
                    chalk_reflectance[wavelengths[i]] = val
                    i += 1
    
    hematite_reflectance = get_reflectance("../genie_calibration_data/s07_ASD_Hematite_GDS69.c_60-104um_BECKb_AREF.txt")
    magnetite_reflectance = get_reflectance("../genie_calibration_data/s07_ASD_Magnetite_HS195.3B_BECKb_AREF.txt")
    serpentine_reflectance = get_reflectance("../genie_calibration_data/s07_ASD_Magnetite_HS195.3B_BECKb_AREF.txt")
    chlorite_reflectance = get_reflectance("../genie_calibration_data/s07_ASD_Magnetite_HS195.3B_BECKb_AREF.txt")

    A = [0] * 12
    B = [0] * 12
    for i in range(12):
        a = [
            [chalk_reflectance[wavelengths[i]], 1],
            [hematite_reflectance[wavelengths[i]], 1],
            [magnetite_reflectance[wavelengths[i]], 1],
            [serpentine_reflectance[wavelengths[i]], 1],
            [chlorite_reflectance[wavelengths[i]], 1]
        ]
        A[i] = a
        B[i] = [0] * 5
        B[i][0] = cal_corrected[i, chalk_roi[1]:chalk_roi[3], chalk_roi[0]:chalk_roi[2]].mean()
        B[i][1] = cal_corrected[i, hematite_roi[1]:hematite_roi[3], hematite_roi[0]:hematite_roi[2]].mean()
        B[i][2] = cal_corrected[i, magnetite_roi[1]:magnetite_roi[3], magnetite_roi[0]:magnetite_roi[2]].mean()
        B[i][3] = cal_corrected[i, serpentine_roi[1]:serpentine_roi[3], serpentine_roi[0]:serpentine_roi[2]].mean()
        B[i][4] = cal_corrected[i, chlorite_roi[1]:chlorite_roi[3], chlorite_roi[0]:chlorite_roi[2]].mean()
        B[i] = np.transpose(B[i])

    M = [0] * 12
    C = [0] * 12
    for i in range(12):
        m, c = np.linalg.lstsq(A[i], B[i])
        M[i] = m
        C[i] = c

    with open("../genie_calibration_data/calibration_constants.csv", "w") as f:
        for i in range(12):
            f.write(str(wavelengths[i]) + ", " + str(M[i]) + ", " + str(C[i]) + "\n")

    reflectance = sample_corrected * M[0] + C[0]
    reflectance_corr = np.divide(reflectance, flat_norm)

    # extract ROI (optional) and compute mean reflectance per wavelength
    if roi is not None:
        x1, y1, x2, y2 = roi  # ROI as (x1, y1, x2, y2)
        reflectance_roi = reflectance_corr[:, y1:y2, x1:x2]
    else:
        reflectance_roi = reflectance_corr

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
    bias_folder = "../genie_cam_calibration/bias"
    dark_folder = "../genie_cam_calibration/dark"
    flat_folder = "../genie_cam_calibration/flat"
    calibration_folder = "../genie_cam_calibration/calibration_target"
    sample_folder = "../genie_cam_calibration/sample"

    # optionally, set ROI as (x1, y1, x2, y2)
    roi = (100, 100, 400, 400)
    chalk_roi = (100, 100, 400, 400)
    hematite_roi = (100, 100, 400, 400)
    magnetite_roi = (100, 100, 400, 400)
    serpentine_roi = (100, 100, 400, 400)
    chlorite_roi = (100, 100, 400, 400)

    mean_reflectance = calibrate_images(
        bias_folder, dark_folder, flat_folder, calibration_folder, sample_folder, wavelengths, roi, chalk_roi, hematite_roi, magnetite_roi, serpentine_roi, chlorite_roi
    )