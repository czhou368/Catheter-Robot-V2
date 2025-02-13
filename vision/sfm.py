from importlib import import_module
from pprint import pprint

from deep_image_matching import logger, timer
from deep_image_matching.config import Config
from deep_image_matching.image_matching import ImageMatching
from deep_image_matching.io.h5_to_db import export_to_colmap

import yaml
import numpy as np

import vision.cam_intrinsics
from pathlib import Path

# # Get the list of possible configurations and chose one of them

# print("Available configurations:")
# pprint(Config.get_pipelines())
# print("Available matching strategy:")
# pprint(Config.get_matching_strategies())

# Available configurations:
['superpoint+lightglue',
 'superpoint+lightglue_fast',
 'superpoint+superglue',
 'disk+lightglue',
 'aliked+lightglue',
 'orb+kornia_matcher',
 'sift+kornia_matcher',
 'loftr',
 'se2loftr',
 'roma',
 'keynetaffnethardnet+kornia_matcher',
 'dedode+kornia_matcher']

# Available matching strategy:
['bruteforce',
 'sequential',
 'retrieval',
 'custom_pairs',
 'matching_lowres',
 'covisibility']

        
def structure_from_motion(dir, pipeline="superpoint+superglue", strategy="bruteforce"):
    cli_params = {
        "dir": dir,
        "images": dir + "/images",
        "pipeline": pipeline,
        "strategy": strategy,
        "quality": "high",
        # "tiling": "preselection",
        "tiling": "none",
        "skip_reconstruction": False,
        "force": True,
        "camera_options": "./config/cameras.yaml",
        "openmvg": None,
        "overlap": 6,
    }
    config = Config(cli_params)
    
    
    # - General configuration
    config.general["min_inliers_per_pair"] = 10
    config.general["min_inlier_ratio_per_pair"] = 0.2

    # - SuperPoint configuration
    config.extractor["max_keypoints"] = 800

    # - LightGue configuration
    config.matcher["filter_threshold"] = 0.1

    # Save configuration to a json file in the output directory
    config.save()

    imgs_dir = config.general["image_dir"]
    # output_dir = config.general["output_dir"]
    output_dir = Path(dir, "sfm")
    matching_strategy = config.general["matching_strategy"]
    extractor = config.extractor["name"]
    matcher = config.matcher["name"]

    # Initialze the ImageMatching class

    img_matching = ImageMatching(
        imgs_dir=imgs_dir,
        output_dir=output_dir,
        matching_strategy=matching_strategy,
        local_features=extractor,
        matching_method=matcher,
        pair_file=config.general["pair_file"],
        retrieval_option=config.general["retrieval"],
        overlap=config.general["overlap"],
        existing_colmap_model=config.general["db_path"],
        custom_config=config.as_dict(),
    )

    pair_path = img_matching.generate_pairs()
    timer.update("generate_pairs")

    # Rotate images so thay will be all "upright"
    # Useful for deep-learning approaches that usually are not rotation invariant

    # if config.general["upright"]:
    #     img_matching.rotate_upright_images()
    #     timer.update("rotate_upright_images")

    # Extract features
    feature_path = img_matching.extract_features()
    timer.update("extract_features")

    # Run matching
    match_path = img_matching.match_pairs(feature_path)
    timer.update("matching")

    # Bing features back to original orientations
    # if config.general["upright"]:
    #     img_matching.rotate_back_features(feature_path)
    #     timer.update("rotate_back_features")


    # # Assign camera models with a yaml alternatively
    # with open(config.general["camera_options"], "r") as file:
    #     camera_options = yaml.safe_load(file)

    camera_options = {
        "general": {
            "camera_model": "opencv",  # ["simple-pinhole", "pinhole", "simple-radial", "opencv"]
            "single_camera": True,
            "param_arr": vision.cam_intrinsics.camera_instrincs_1,
        },
    }

    database_path = output_dir / "database.db"
    export_to_colmap(
        img_dir=imgs_dir,
        feature_path=feature_path,
        match_path=match_path,
        database_path=database_path,
        camera_options=camera_options,
    )
    timer.update("export_to_colmap")

    # Run construction

    if not config.general["skip_reconstruction"]:
        use_pycolmap = True
        try:
            pycolmap = import_module("pycolmap")
        except ImportError:
            logger.error("Pycomlap is not available.")
            use_pycolmap = False

    if not config.general["skip_reconstruction"] and use_pycolmap:
        # import reconstruction module
        from deep_image_matching import reconstruction

        # Define database path and camera mode
        database = output_dir / "database.db"
        # camera_mode = pycolmap.CameraMode.AUTO
        # cameras = None
        reconst_opts = {
            "ba_refine_focal_length": False,
            "ba_refine_principal_point": False,
            "ba_refine_extra_params": False,
        }
        model = reconstruction.main(
            database=database,
            image_dir=imgs_dir,
            feature_path=feature_path,
            match_path=match_path,
            pair_path=pair_path,
            sfm_dir=output_dir,
            # camera_mode=camera_mode,
            # cameras=cameras,
            skip_geometric_verification=True,
            reconst_opts=reconst_opts,
            verbose=config.general["verbose"],
        )

        timer.update("pycolmap reconstruction")

    # Print COLMAP camera values
    # Unchanged here
    print(list(model.cameras.values()))

    # mvs_path = output_dir / "mvs"
    # reconstruction_path = output_dir / "reconstruction"
    # pycolmap.undistort_images(mvs_path, reconstruction_path, imgs_dir)
    # timer.update("undistort_images")
    # print("Finish image undistortion!")
    # pycolmap.patch_match_stereo(mvs_path)
    # print("Finish stereo match!")
    # pycolmap.stereo_fusion(mvs_path / "dense.ply", mvs_path)
    # print("Finish dense reconstruction!")
