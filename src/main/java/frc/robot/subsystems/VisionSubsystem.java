package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelights.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final String llName = "limelight";
    private NetworkTable table;


    public VisionSubsystem() {
        this.table = NetworkTableInstance.getDefault().getTable(llName);
    }

    /**
     * @return true if the Limelight sees any valid target.
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(llName);
    }

    /**
     * Use this for "Chasing" game pieces.
     * @return The class name of the primary detected object (e.g., "Algae").
     */
    public String getDetectedClass() {
        String rawJson = table.getEntry("json").getString("");
        if (!rawJson.isEmpty()) {
            System.out.println("RAW DATA: " + rawJson);
        }
        var results = LimelightHelpers.getLatestResults(llName).targets_Detector;
        // Your JSON shows data in targets_Detector
        if (results != null && results.length > 0) {
            // Use .get(0) to access the primary target's data
            return results[0].className;
        }        return "None";
    }

    /**
     * Use this for "Shooting" alignment.
     * @return Horizontal offset in degrees.
     */
    public double getTX() {
        return LimelightHelpers.getTX(llName);
    }

    /**
     * Use this for distance estimation to the target.
     */
    public double getTY() {
        return LimelightHelpers.getTY(llName);
    }

    /**
     * Returns the area of the target as a percentage of the image.
     * Useful for determining if you are "close enough" to intake.
     */
    public double getTargetArea() {
        return LimelightHelpers.getTA(llName);
    }

    @Override
    public void periodic() {
        // Optional: Post results to SmartDashboard for driver feedback
    }
}
