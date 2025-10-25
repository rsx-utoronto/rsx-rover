from roboflow import Roboflow
import os

# Initialize the Roboflow object with your API key
rf = Roboflow(api_key="SwKLHL06BGIaLT2rAlpb")

# Retrieve your current workspace and project name
print(rf.workspace())

# Specify the project for upload
# let's you have a project at https://app.roboflow.com/my-workspace/my-project
workspaceId = 'rsxobjectdetection'
projectId = 'favoriteappetizer-sftcy'
project = rf.workspace(workspaceId).project(projectId)

folders = [
    r"C:\Users\Jakkii\Downloads\Pickaxe_Data\Image Detection Hammer 3.0"
]

image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".gif", ".heic"}

uploaded_count = 0
for folder in folders:
    if not os.path.isdir(folder):
        print(f"Skipping missing folder: {folder}")
        continue
    for root, _, files in os.walk(folder):
        for name in files:
            if os.path.splitext(name)[1].lower() in image_exts:
                image_path = os.path.join(root, name)
                try:
                    project.upload(image_path=image_path)
                    uploaded_count += 1
                    print(f"Uploaded: {image_path}")
                except Exception as e:
                    print(f"Failed to upload {image_path}: {e}")

print(f"Total images uploaded: {uploaded_count}")
# Upload the image to your project
#project.upload("UPLOAD_IMAGE.jpg")

"""
Optional Parameters:
- num_retry_uploads: Number of retries for uploading the image in case of failure.
- batch_name: Upload the image to a specific batch.
- split: Upload the image to a specific split.
- tag: Store metadata as a tag on the image.
- sequence_number: [Optional] If you want to keep the order of your images in the dataset, pass sequence_number and sequence_size..
- sequence_size: [Optional] The total number of images in the sequence. Defaults to 100,000 if not set.
"""

#project.upload(
#   image_path="UPLOAD_IMAGE.jpg",
#    batch_name="YOUR_BATCH_NAME",
#    split="train",
#    num_retry_uploads=3,
#    tag_names=["YOUR_TAG_NAME"],
#    sequence_number=99,
#    sequence_size=100
#)