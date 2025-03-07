from vision.SAM_preprocessing import SAMObjectTracker
from vision.sfm import structure_from_motion
from vision.visualize_model import SfM_visualization
from vision.gsplat import Config, run_gsplat
import torch
import cv2

RUN_SAM = 0
RUN_SFM = 0
RUN_VISUALIZATION = 0
RUN_GSPLAT = 1

if __name__ == "__main__":
    # select the device for computation
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
    print(f"using device: {device}")

    if device.type == "cuda":
        # use bfloat16 for the entire notebook
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
    
    dir = "./Data/test_1"
    
    if RUN_SAM:
        # create the SAM object tracker
        sam_object_tracker = SAMObjectTracker(device=device, dir=dir, sam2_dir="../sam2", write_segmented_images=True)
        sam_object_tracker.interact_with_first_image()
        images = sam_object_tracker.track_object()
        
    if RUN_SFM:
        input("Press Enter to run SfM...")
        cv2.destroyAllWindows()
        # Structure from Motion
        structure_from_motion(dir=dir)
    
    if RUN_VISUALIZATION:
        input("Press Enter to visualize the SfM result...")
        # visualize the SfM model
        SfM_visualization(input_model=dir + "/sfm/models/0", input_format=".bin")
        
    if RUN_GSPLAT:
        input("Press Enter to run gaussian splatting...")
        # run gaussian spaltting
        run_gsplat(data_dir=dir, max_steps=10000)