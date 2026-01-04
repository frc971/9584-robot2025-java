package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelights.Constants;
import frc.robot.subsystems.Limelights.LimelightHelpers;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private final String objectDetectorName = Constants.LimelightConstants.OBJECT_DETECTOR_NAME;
    private final List<String> aprilTagCameras = Constants.LimelightConstants.APRILTAG_CAMERA_NAMES;

    private double m_autonomousStartTime = -1.0;

    public VisionSubsystem() {}

    /**
     * Call this in autonomousInit() via the Robot class or a Command
     */
    public void resetAutoTimer() {
        m_autonomousStartTime = Timer.getFPGATimestamp() * 1000.0;
    }

    public void updateOdometry(CommandSwerveDrivetrain drivetrain) {
        var driveState = drivetrain.getState();
        double heading = driveState.Pose.getRotation().getDegrees();
        double omega = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
        double currentTime = Timer.getFPGATimestamp() * 1000.0;

        // Safety 1: Re-implementing the 200ms Auto-Start delay from your Robot.java
        if (m_autonomousStartTime > 0 && currentTime - m_autonomousStartTime < 200.0) {
            return;
        }

        for (String name : aprilTagCameras) {
            LimelightHelpers.SetRobotOrientation(name, heading, omega, 0, 0, 0, 0);

            var alliance = DriverStation.getAlliance();
            LimelightHelpers.PoseEstimate measure = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                    ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

            // Safety 2: Re-implementing the spin-filter and tag-count check
            if (measure != null && measure.tagCount > 0 && Math.abs(omega) < 720.0) {

                // Safety 3: Re-implementing your Dynamic StdDev logic
                // Your old code: xyStdDev = avgTagDist * 0.5;
                double trustFactor = (measure.tagCount >= 2) ? 0.25 : 0.5;
                double xyStdDev = measure.avgTagDist * trustFactor;

                // We use 999999 for rotation to trust the Pigeon2 gyro over vision
                drivetrain.addVisionMeasurement(
                        measure.pose,
                        measure.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, 999999)
                );

                // Telemetry: Moved from Robot.java to here
                SmartDashboard.putNumber("Vision/" + name + "/TagCount", measure.tagCount);
                SmartDashboard.putNumber("Vision/" + name + "/AvgDist", measure.avgTagDist);
            }
        }
    }

    @Override
    public void periodic() {
        // Re-implementing updateLimelightTelemetry() from your Robot.java
        // This keeps the dashboard updated even if odometry isn't being fused
        updateDashboardTelemetry(objectDetectorName);
        for(String name : aprilTagCameras) {
            updateDashboardTelemetry(name);
        }
    }

    private void updateDashboardTelemetry(String name) {
        boolean hasTarget = LimelightHelpers.getTV(name);
        SmartDashboard.putBoolean(name + "/HasTarget", hasTarget);
        if (hasTarget) {
            SmartDashboard.putNumber(name + "/TX", LimelightHelpers.getTX(name));
            SmartDashboard.putNumber(name + "/TY", LimelightHelpers.getTY(name));
        }
    }

    // --- Neural Detector Accessors ---
    public boolean hasTarget() {
        return LimelightHelpers.getTV(objectDetectorName);
    }

    public String getDetectedClass() {
        var results = LimelightHelpers.getLatestResults(objectDetectorName).targets_Detector;
        return (results != null && results.length > 0) ? results[0].className : "None";
    }

    public double getTX() { return LimelightHelpers.getTX(objectDetectorName); }
    public double getTY() { return LimelightHelpers.getTY(objectDetectorName); }
    public double getTargetArea() { return LimelightHelpers.getTA(objectDetectorName); }
}