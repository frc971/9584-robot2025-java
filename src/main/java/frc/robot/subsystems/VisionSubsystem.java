package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelights.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    // LL4 Names - Match these to your Limelight hostnames
    private final String objectDetectorName = "limelight";
    private final String[] aprilTagCameras = {"limelight-front", "limelight-back"};

    public VisionSubsystem() {}

    /**
     * Updates the robot's odometry by fusing data from AprilTag cameras using MegaTag2.
     * @param drivetrain The drivetrain instance from RobotContainer.
     */
    public void updateOdometry(CommandSwerveDrivetrain drivetrain) {
        var driveState = drivetrain.getState();
        double heading = driveState.Pose.getRotation().getDegrees();
        // MegaTag2 requires angular velocity in degrees/sec
        double omega = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);

        for (String name : aprilTagCameras) {
            // Set orientation for precise 3D localization
            LimelightHelpers.SetRobotOrientation(name, heading, omega, 0, 0, 0, 0);

            var alliance = DriverStation.getAlliance();
            LimelightHelpers.PoseEstimate measure;

            // Select the correct alliance pose
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                measure = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
            } else {
                measure = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            }

            // Only fuse if we see tags and rotation isn't excessive
            if (measure != null && measure.tagCount > 0 && Math.abs(omega) < 720.0) {
                // Trust 2+ tags significantly more than 1 tag
                double trustFactor = (measure.tagCount >= 2) ? 0.25 : 0.7;
                double stdDev = measure.avgTagDist * trustFactor;

                // Call the bridge method added to CommandSwerveDrivetrain
                drivetrain.addVisionMeasurement(
                        measure.pose,
                        measure.timestampSeconds,
                        VecBuilder.fill(stdDev, stdDev, 999999) // High theta value to trust gyro
                );
            }
        }
    }

    // --- Object Detection APIs (Using Working Structure) ---

    public boolean hasTarget() {
        return LimelightHelpers.getTV(objectDetectorName);
    }

    public String getDetectedClass() {
        // Using the confirmed working API path
        var results = LimelightHelpers.getLatestResults(objectDetectorName).targets_Detector;

        if (results != null && results.length > 0) {
            return results[0].className;
        }
        return "None";
    }

    public double getTX() { return LimelightHelpers.getTX(objectDetectorName); }
    public double getTY() { return LimelightHelpers.getTY(objectDetectorName); }
    public double getTargetArea() { return LimelightHelpers.getTA(objectDetectorName); }

    @Override
    public void periodic() {
        // Runs 50Hz on the robot for optional logging
    }
}