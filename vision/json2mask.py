import json
import numpy as np
import cv2
import os

image_size = (360, 640) # height, width

# Step 1: Load the LabelMe JSON file
def json_to_mask(json_file, output_dir, img_size):
    with open(json_file) as f:
        data = json.load(f)

    # Step 2: Create an empty mask image
    mask = np.zeros(img_size, dtype=np.uint8)

    # Step 3: Loop through the shapes (polygons) in the JSON file
    for shape in data['shapes']:
        points = shape['points']  # list of (x, y) points
        polygon = np.array(points, dtype=np.int32)

        # Step 4: Draw the polygon on the mask image
        cv2.fillPoly(mask, [polygon], 255)  # 255 means white region for the mask
    
    # Step 5: Save the binary mask image
    mask_filename = os.path.join(output_dir, f"{os.path.splitext(os.path.basename(json_file))[0]}_mask.png")
    cv2.imwrite(mask_filename, mask)
    print(f"Mask saved at {mask_filename}")

if __name__ == "__main__":

    dir = './Data/skull/'
    
    json_dir = dir + 'annotations/' # input dir
    mask_dir = dir + 'masks/' # output dir

    for json_file in os.listdir(json_dir):
        if json_file.endswith('.json'):
            
            json_to_mask(json_dir + json_file, mask_dir, image_size)