package frc.robot.subsystems.Limelights;

import java.util.List;

public class Constants {
    public static class LimelightConstants {
        // Updated to match your fleet of three Limelight 4s
        public static final String OBJECT_DETECTOR_NAME = "limelight";
        public static final List<String> APRILTAG_CAMERA_NAMES = List.of("limelight-front", "limelight-back");

        // Combined list for initialization loops in Robot.java
        public static final List<String> limelightNames = List.of("limelight", "limelight-front", "limelight-back");

        // Physical Mounting (Update these based on your actual robot CAD)
        public static final double CAMERA_HEIGHT_METERS = 0.5;
        public static final double CAMERA_PITCH_DEGREES = 25.0;

        // Trust and Filtering Constants
        public static final double VISION_STD_DEV_COEFFICIENT = 0.5;
        public static final double MULTI_TAG_STD_DEV_MULTIPLIER = 0.25; // More trust for 2+ tags

        // MegaTag2 is stable at higher speeds than legacy logic
        public static final double MAX_ANGULAR_VELOCITY_FOR_VISION_DEG = 720.0;
        public static final double AUTO_VISION_DELAY_MS = 200.0;

        // Pipeline IDs (Set in Limelight Web Dashboard)
        public static final int NEURAL_DETECTOR_PIPELINE = 0;
        public static final int APRILTAG_PIPELINE = 0;
    }
}