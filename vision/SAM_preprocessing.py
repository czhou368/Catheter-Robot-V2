# This code uses SAM 2 to segment and track one object in a series of N images.
# Use your mouse to click on the object in interest in the first image (two different points for robustness),
# and SAM 2 will automatically track and segment the object in the remained N - 1 images.

import os
import numpy as np
import torch
import cv2

from sam2.build_sam import build_sam2_video_predictor

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


class SAMObjectTracker:
    def __init__(
        self,
        device,
        dir="../Data/puppy_w_distraction",
        video_dir="/ori_images",  # `video_dir` a directory of JPEG frames with filenames like `<frame_index>.jpg`
        mode="rectangle",  # choose between 'points' and 'rectangle'
        sam2_dir="../../sam2",
        sam2_checkpoint="/checkpoints/sam2.1_hiera_tiny.pt",
        model_cfg="configs/sam2.1/sam2.1_hiera_t.yaml",
        write_masks=True,
        write_segmented_images=True,
    ):
        self.write_masks = write_masks
        self.write_segmented_images = write_segmented_images
        
        self.sam2_dir = sam2_dir
        self.sam2_checkpoint = self.sam2_dir + sam2_checkpoint
        self.model_cfg = model_cfg
        self.device = device
        
        self.predictor = build_sam2_video_predictor(
            self.model_cfg, self.sam2_checkpoint, device=self.device
        )

        self.dir = dir
        self.video_dir = dir + video_dir

        # scan all the JPEG frame names in this directory
        self.frame_names = [
            p
            for p in os.listdir(self.video_dir)
            if os.path.splitext(p)[-1] in [".jpg", ".jpeg", ".JPG", ".JPEG"]
        ]
        self.frame_names.sort(key=lambda p: int(os.path.splitext(p)[0]))

        print("Found %d frames in the video directory." % len(self.frame_names))
        
        # take a look the first video frame
        self.frame_idx = 0

        self.first_image = (
            cv2.imread(
                os.path.join(self.video_dir, self.frame_names[self.frame_idx])
            ).astype(np.float32)
            / 255
        )

        self.width = self.first_image.shape[1]
        self.height = self.first_image.shape[0]

        self.inference_state = self.predictor.init_state(video_path=self.video_dir)
        # predictor.reset_state(inference_state)

        self.ann_frame_idx = 0  # the frame index we interact with
        self.ann_obj_id = 1  # give a unique id to each object we interact with (it can be any integers)

        self.mode = mode

        self.windowname = ""

        self.first_mask = None

        # self.segmented_images = []
        
    def interact_with_first_image(self):
        self.windowname = "First Image"

        print("Using interactive method:", self.mode)
        
        if self.mode == "points":
            coordinateStore = MousePts(self.windowname, self.first_image)
            points, _ = coordinateStore.getpt(2)
            print("Slected point(s):", points)

            # for labels, `1` means positive click and `0` means negative click
            labels = np.array([1, 1], np.int32)
            _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
                inference_state=self.inference_state,
                frame_idx=self.ann_frame_idx,
                obj_id=self.ann_obj_id,
                points=points,
                labels=labels,
            )

            self.first_mask = (
                (out_mask_logits[0] > 0.0).cpu().numpy().astype(np.float32)
            )
            self.first_mask = np.reshape(self.first_mask, [self.width, self.height, 1])

            show_mask_and_points(self.first_image, self.first_mask, points, index=0)

        elif self.mode == "rectangle":
            rectangleStore = MouseRect(self.windowname, self.first_image)
            upperleft, bottomright, _ = rectangleStore.getrect()
            # print("Slected point(s):", points)
            # print(upperleft)
            # print(bottomright)

            # for labels, `1` means positive click and `0` means negative click
            box = np.array(
                [upperleft[0], upperleft[1], bottomright[0], bottomright[1]],
                dtype=np.float32,
            )
            _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
                inference_state=self.inference_state,
                frame_idx=self.ann_frame_idx,
                obj_id=self.ann_obj_id,
                box=box,
            )

            self.first_mask = (
                (out_mask_logits[0] > 0.0).cpu().numpy().astype(np.float32)
            )
            self.first_mask = np.reshape(self.first_mask, [self.width, self.height, 1])

            show_mask_and_rectangle(
                self.first_image, self.first_mask, upperleft, bottomright, index=0
            )
        else:
            print("Wrong interactive method! Choose between 'points' and 'rectangle'.")
            return

    def track_object(self):
        # run propagation throughout the video and collect the results in a dict
        video_segments = {}  # video_segments contains the per-frame segmentation results
        for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(
            self.inference_state
        ):
            video_segments[out_frame_idx] = {
                out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
                for i, out_obj_id in enumerate(out_obj_ids)
            }
        
        # render the segmentation results every few frames
        vis_frame_stride = 1
        
        segmented_images = []
        
        for out_frame_idx in range(0, len(self.frame_names), vis_frame_stride):

            ori_image = (
                cv2.imread(os.path.join(self.video_dir, self.frame_names[out_frame_idx])).astype(
                    np.float32
                )
                / 255
            )

            for out_obj_id, out_mask in video_segments[out_frame_idx].items():
                h, w = out_mask.shape[-2:]
                mask_image = out_mask.reshape(h, w, 1).astype(np.float32)

                masked_image = mask_image * ori_image

                if self.write_masks:
                    cv2.imwrite(self.dir + "/masks/%04d.png" % out_frame_idx, mask_image * 255)
                    print("Writing mask #%d" % out_frame_idx)
                if self.write_segmented_images:
                    cv2.imwrite(
                        self.dir + "/images/%04d.png" % out_frame_idx, masked_image * 255
                    )
                    print("Writing segmented image #%d" % out_frame_idx)

                segmented_images.append(masked_image)
                
                show_mask(ori_image, mask_image, index=out_frame_idx, pause=False)
   
        return segmented_images
    
def show_mask(
    image,
    mask,
    index=0,
    size=(WIDTH, HEIGHT),
    mask_color=(255, 0, 255),
    window_name="Segmentation Result",
    pause=True,
    delay=200,
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
    if pause:
        print("Press any key to conitnue.")
        cv2.waitKey(0)
    else:
        cv2.waitKey(delay)


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
    print("Press any key to conitnue.")
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

    cv2.rectangle(
        combine, point_ul, point_br, color=rectangle_color, thickness=thickness
    )

    cv2.imshow(window_name, combine)
    print("Showing the segmentation result of image #%d." % index)
    print("Press any key to conitnue.")
    cv2.waitKey(0)

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

    tracker = SAMObjectTracker()
    tracker.interact_with_first_image()
    tracker.track_object()
