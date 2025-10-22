package frc.robot;

import java.util.List;

public class Constants {
    public static class LimelightConstants {
        public static final List<String> limelightNames = List.of("limelight-down");
        
        public static final double CAMERA_HEIGHT_METERS = 0.5;
        public static final double CAMERA_PITCH_DEGREES = 25.0;
        
        public static final double VISION_STD_DEV_COEFFICIENT = 0.5;
        public static final double MULTI_TAG_STD_DEV_MULTIPLIER = 0.5;
        public static final double MAX_ANGULAR_VELOCITY_FOR_VISION = 2.0;
        
        public static final double MIN_TAG_AREA = 0.1;
        public static final double MAX_AMBIGUITY = 0.2;
        public static final double AUTO_VISION_DELAY_MS = 200.0;
        
        public static final int APRILTAG_PIPELINE = 0;
        public static final int RETROREFLECTIVE_PIPELINE = 1;
        public static final int DRIVER_CAMERA_PIPELINE = 9;
        
        public static final double SPEAKER_TARGET_HEIGHT_METERS = 2.0;
        public static final double AMP_TARGET_HEIGHT_METERS = 1.37;
    }
}