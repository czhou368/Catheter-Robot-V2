# This code uses SAM 2 to segment and track one object in a series of N images.
# Use your mouse to click on the object in interest in the first image (two different points for robustness),
# and SAM 2 will automatically track and segment the object in the remained N - 1 images.

import os
import numpy as np
import torch
import cv2

from sam2.build_sam import build_sam2_video_predictor

INTERACTIVE_METHOD = "rectangle"
WRITE_MASKS = True
WRITE_SEGMENTED_IMAGES = True
WIDTH = 400
HEIGHT = 400

class MousePts:
    def __init__(self, windowname, img):
        self.windowname = windowname
        self.img1 = img.copy()
        self.img = self.img1.copy()
        cv2.namedWindow(self.windowname)
        cv2.imshow(self.windowname, img)
        self.curr_pt = []
        self.point = []

    def select_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point.append([x, y])
            # print(self.point)
            cv2.circle(self.img, (x, y), 5, (0, 255, 0), -1)
        elif event == cv2.EVENT_MOUSEMOVE:
            self.curr_pt = [x, y]
            # print(self.point)

    def getpt(self, count=1, img=None):
        if img is not None:
            self.img = img
        else:
            self.img = self.img1.copy()
        cv2.namedWindow(self.windowname)
        
        self.point = []

        while 1:
            # cv2.imshow(self.windowname, self.img)
            cv2.setMouseCallback(self.windowname, self.select_point)
            cv2.imshow(self.windowname, self.img)
            k = cv2.waitKey(20) & 0xFF
            if k == 27 or len(self.point) >= count:
                break
            # print(self.point)

        # cv2.setMouseCallback(self.windowname, lambda *args: None)
        # cv2.destroyAllWindows()
        return self.point, self.img


class MouseRect:
    def __init__(self, windowname, img):
        self.windowname = windowname
        self.img1 = img.copy()
        self.img = self.img1.copy()
        cv2.namedWindow(self.windowname)
        cv2.imshow(self.windowname, img)
        self.startP = []
        self.endP = []
        self.flag = -1
        self.currentP = []
        self.img2 = self.img1.copy()
        
    def draw_rectangle(self, event, x, y, flags, param):

        self.img2 = self.img.copy()
        
        if event == cv2.EVENT_LBUTTONDOWN:
            self.flag = 1
            self.startP = [x, y]

        if event == cv2.EVENT_LBUTTONUP:
            self.flag = 0
            self.endP = [x, y]
            cv2.rectangle(self.img, self.startP, self.endP, [0, 0, 255])

        if event == cv2.EVENT_MOUSEMOVE:
            self.currentP = [x, y]
            if self.flag == 1:
                cv2.rectangle(self.img2, self.startP, self.currentP, [0, 0, 255])
                cv2.imshow(self.windowname, self.img2)
        
        if event == cv2.EVENT_MOUSEWHEEL:
            print(x, y)

    def getrect(self, img=None):
        if img is not None:
            self.img = img
        else:
            self.img = self.img1.copy()
        cv2.namedWindow(self.windowname)
               
        while 1:
            cv2.setMouseCallback(self.windowname, self.draw_rectangle)
            cv2.imshow(self.windowname, self.img2)
            k = cv2.waitKey(20) & 0xFF
            if k == 27 or self.flag == 0:
                break
            # print(self.point)

        # cv2.setMouseCallback(self.windowname, lambda *args: None)
        # cv2.destroyAllWindows()

        return self.startP, self.endP, self.img


def show_mask(
    image,
    mask,
    index=0,
    size=(WIDTH, HEIGHT),
    mask_color=(255, 0, 255),
    window_name="Segmentation Result",
):
    mask = np.concatenate(
        [
            mask * mask_color[0] / 255,
            mask * mask_color[1] / 255,
            mask * mask_color[2] / 255,
        ],
        axis=2,
    )
    combine = cv2.addWeighted(
        cv2.resize(image, size), 1.0, cv2.resize(mask, size), 0.3, 0
    )
    cv2.imshow(window_name, combine)
    print("Showing the segmentation result of image #%d." % index)
    cv2.waitKey(0)


def show_mask_and_points(
    image,
    mask,
    points,
    index=0,
    size=(WIDTH, HEIGHT),
    mask_color=(255, 0, 255),
    point_color=(0, 255, 0),
    window_name="First Image",
):
    mask = np.concatenate(
        [
            mask * mask_color[0] / 255,
            mask * mask_color[1] / 255,
            mask * mask_color[2] / 255,
        ],
        axis=2,
    )
    combine = cv2.addWeighted(
        cv2.resize(image, size), 1.0, cv2.resize(mask, size), 0.3, 0
    )

    point_size = 3
    thickness = 4

    for point in points:
        cv2.circle(combine, point, point_size, point_color, thickness)

    cv2.imshow(window_name, combine)
    print("Showing the segmentation result of image #%d." % index)
    cv2.waitKey(0)

def show_mask_and_rectangle(
    image,
    mask,
    point_ul,
    point_br,
    index=0,
    size=(WIDTH, HEIGHT),
    mask_color=(255, 0, 255),
    rectangle_color=(0, 255, 0),
    window_name="First Image",
):
    mask = np.concatenate(
        [
            mask * mask_color[0] / 255,
            mask * mask_color[1] / 255,
            mask * mask_color[2] / 255,
        ],
        axis=2,
    )
    combine = cv2.addWeighted(
        cv2.resize(image, size), 1.0, cv2.resize(mask, size), 0.3, 0
    )

    thickness = 2


    cv2.rectangle(combine, point_ul, point_br, color=rectangle_color, thickness=thickness)

    cv2.imshow(window_name, combine)
    print("Showing the segmentation result of image #%d." % index)
    cv2.waitKey(0)

def main():

    sam2_dir = "../../sam2"
    sam2_checkpoint = sam2_dir + "/checkpoints/sam2.1_hiera_tiny.pt"
    model_cfg = "configs/sam2.1/sam2.1_hiera_t.yaml"

    predictor = build_sam2_video_predictor(model_cfg, sam2_checkpoint, device=device)

    # `video_dir` a directory of JPEG frames with filenames like `<frame_index>.jpg`
    dir = "../Data/puppy_w_distraction"
    video_dir = dir + "/ori_images"

    # scan all the JPEG frame names in this directory
    frame_names = [
        p
        for p in os.listdir(video_dir)
        if os.path.splitext(p)[-1] in [".jpg", ".jpeg", ".JPG", ".JPEG"]
    ]
    frame_names.sort(key=lambda p: int(os.path.splitext(p)[0]))

    # take a look the first video frame
    frame_idx = 0

    first_image = (
        cv2.imread(os.path.join(video_dir, frame_names[frame_idx])).astype(np.float32)
        / 255
    )

    inference_state = predictor.init_state(video_path=video_dir)
    # predictor.reset_state(inference_state)

    ann_frame_idx = 0  # the frame index we interact with
    ann_obj_id = (
        1  # give a unique id to each object we interact with (it can be any integers)
    )

    # Manually interact
    windowname = "First Image"
    
    if INTERACTIVE_METHOD == "points":
        coordinateStore = MousePts(windowname, first_image)
        points, _ = coordinateStore.getpt(2)
        print("Slected point(s):", points)

        # for labels, `1` means positive click and `0` means negative click
        labels = np.array([1, 1], np.int32)
        _, out_obj_ids, out_mask_logits = predictor.add_new_points_or_box(
            inference_state=inference_state,
            frame_idx=ann_frame_idx,
            obj_id=ann_obj_id,
            points=points,
            labels=labels,
        )
        
        first_mask = (out_mask_logits[0] > 0.0).cpu().numpy().astype(np.float32)
        first_mask = np.reshape(first_mask, [WIDTH, HEIGHT, 1])
        
        show_mask_and_points(first_image, first_mask, points, index=0)
        
    elif INTERACTIVE_METHOD == "rectangle":
        rectangleStore = MouseRect(windowname, first_image)
        upperleft, bottomright, _ = rectangleStore.getrect()
        # print("Slected point(s):", points)
        print(upperleft)
        print(bottomright)
        
        # for labels, `1` means positive click and `0` means negative click
        box = np.array([upperleft[0], upperleft[1], bottomright[0], bottomright[1]], dtype=np.float32)
        _, out_obj_ids, out_mask_logits = predictor.add_new_points_or_box(
            inference_state=inference_state,
            frame_idx=ann_frame_idx,
            obj_id=ann_obj_id,
            box=box,
        )
        
        first_mask = (out_mask_logits[0] > 0.0).cpu().numpy().astype(np.float32)
        first_mask = np.reshape(first_mask, [WIDTH, HEIGHT, 1])

        show_mask_and_rectangle(first_image, first_mask, upperleft, bottomright, index=0)
    else:
        print("Wrong interactive method! Choose between 'points' and 'rectangle'.")    
        return
              
        

    # run propagation throughout the video and collect the results in a dict
    video_segments = {}  # video_segments contains the per-frame segmentation results
    for out_frame_idx, out_obj_ids, out_mask_logits in predictor.propagate_in_video(
        inference_state
    ):
        video_segments[out_frame_idx] = {
            out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
            for i, out_obj_id in enumerate(out_obj_ids)
        }

    # render the segmentation results every few frames
    vis_frame_stride = 1

    for out_frame_idx in range(0, len(frame_names), vis_frame_stride):

        ori_image = (
            cv2.imread(os.path.join(video_dir, frame_names[out_frame_idx])).astype(
                np.float32
            )
            / 255
        )

        for out_obj_id, out_mask in video_segments[out_frame_idx].items():
            h, w = out_mask.shape[-2:]
            mask_image = out_mask.reshape(h, w, 1).astype(np.float32)

            masked_image = mask_image * ori_image

            if WRITE_MASKS:
                cv2.imwrite(dir + "/masks/%04d.png" % out_frame_idx, mask_image * 255)
                print("Writing mask #%d" % out_frame_idx)
            if WRITE_SEGMENTED_IMAGES:
                cv2.imwrite(
                    dir + "/images/%04d.png" % out_frame_idx, masked_image * 255
                )
                print("Writing segmented image #%d" % out_frame_idx)

            show_mask(ori_image, mask_image, index=out_frame_idx)


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

    main()
