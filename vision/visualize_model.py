import argparse

from pcd_model import PointCloudModel

def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize COLMAP binary and text models"
    )
    parser.add_argument(
        "--input_model", required=True, help="path to input model folder"
    )
    parser.add_argument(
        "--input_format",
        choices=[".bin", ".txt"],
        help="input model format",
        default="",
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    # read COLMAP model
    model = PointCloudModel()
    model.read_model(args.input_model, ext=args.input_format)

    print("Number of cameras:", len(model.cameras))
    print("Number of images:", len(model.images))
    print("Number of raw points:", len(model.points3D))

    # display using Open3D visualization tools
    model.create_window()
    xyz, rgb = model.add_points()
    # Segment point cloud using image segmentation information
    # xyz, rgb = model.segment_points(xyz, rgb, 1, './Data/skull/masks/0003_mask.png')
    # xyz, rgb = model.segment_points(xyz, rgb, 2, './Data/skull/masks/0006_mask.png')
    model.points2o3d(xyz, rgb)
    print("Number of points after selection:", len(xyz))
    model.add_cameras(scale=0.25)
    model.show()


if __name__ == "__main__":
    main()