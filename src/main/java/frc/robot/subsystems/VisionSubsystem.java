package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelights.LimelightHelpers; // Note your specific path

public class VisionSubsystem extends SubsystemBase {
    private final String llName = "limelight";

    public VisionSubsystem() {}

    // Get the horizontal offset to the target
    public double getTX() {
        return LimelightHelpers.getTX(llName);
    }

    // Get the vertical offset (useful for distance estimation)
    public double getTY() {
        return LimelightHelpers.getTY(llName);
    }

    // Identify the YOLOv8 class currently being seen
    public String getDetectedClass() {
        return !LimelightHelpers.getNeuralClassID(llName).isEmpty() ?
                LimelightHelpers.getLatestResults(llName).targetingResults.classifierResults.get(0).className :
                "None";
    }

    @Override
    public void periodic() {
        // This runs 50 times a second on the robot
    }
}