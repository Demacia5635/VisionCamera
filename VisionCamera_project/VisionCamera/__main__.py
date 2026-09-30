def main() -> None:
    print(
        "VisionCamera - overhead camera pose/velocity ground truth.\n\n"
        "Sub-tools (also available as console scripts after `pip install -e .`):\n"
        "  python -m VisionCamera.VideoWriter       record the camera stream to MP4\n"
        "  python -m VisionCamera.viewCaptures       browse/merge periodic snapshots\n"
        "  python -m VisionCamera.calibration_tool   build calibration.json from a reference frame\n"
        "  python -m VisionCamera.pose_analyzer      mark 2 frames, compute distance/direction/velocity/omega\n"
    )


if __name__ == "__main__":
    main()
