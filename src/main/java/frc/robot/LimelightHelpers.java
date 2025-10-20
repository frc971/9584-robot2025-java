// LimelightHelpers.java
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.JSONArray;
import org.json.JSONObject;

public class LimelightHelpers {
    public static String sanitizeName(String name) {
        if (name.isEmpty()) {
            return "limelight";
        }
        return name;
    }

    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]), Units.degreesToRadians(inData[5]))
        );
    }

    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
            new Translation2d(inData[0], inData[1]),
            Rotation2d.fromDegrees(inData[5])
        );
    }

    public static void setLEDMode_PipelineControl(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("ledMode").setDouble(0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("ledMode").setDouble(1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("ledMode").setDouble(2);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("ledMode").setDouble(3);
    }

    public static void setStreamMode_Standard(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("stream").setDouble(0);
    }

    public static void setStreamMode_PiPMain(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("stream").setDouble(1);
    }

    public static void setStreamMode_PiPSecondary(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("stream").setDouble(2);
    }

    public static void setCameraMode_Processor(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("camMode").setDouble(0);
    }

    public static void setCameraMode_Driver(String limelightName) {
        getLimelightNTTable(limelightName).getEntry("camMode").setDouble(1);
    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        getLimelightNTTable(limelightName).getEntry("pipeline").setDouble(pipelineIndex);
    }

    public static void setPriorityTagID(String limelightName, int ID) {
        getLimelightNTTable(limelightName).getEntry("priorityid").setDouble(ID);
    }

    public static boolean getIsTargetExist(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "tv").getDouble(0) == 1;
    }

    public static double getTargetPoseX(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "tx").getDouble(0.00);
    }

    public static double getTargetPoseY(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "ty").getDouble(0.00);
    }

    public static double getTargetPitch(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "tp").getDouble(0.00);
    }

    public static double getTargetArea(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "ta").getDouble(0.00);
    }

    public static double getTargetSkew(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "ts").getDouble(0.00);
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "tl").getDouble(0.00);
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "cl").getDouble(0.00);
    }

    public static String getNeuralClassID(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "tclass").getString("");
    }

    public static double getPipelineIndex(String limelightName) {
        return getLimelightNTTableEntry(limelightName, "pipe").getDouble(0);
    }

    public static double[] getBoundingBox(String limelightName) {
        double[] defaultr = new double[4];
        return getLimelightNTTableEntry(limelightName, "tcornxy").getDoubleArray(defaultr);
    }

    public static int getPrimaryAprilTagID(String limelightName) {
        return (int) getLimelightNTTableEntry(limelightName, "tid").getDouble(-1);
    }

    public static double[] getCameraTranslation(String limelightName) {
        double[] defaultr = new double[6];
        return getLimelightNTTableEntry(limelightName, "camtran").getDoubleArray(defaultr);
    }

    public static void setRobotOrientation(String limelightName, double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        double[] arr = new double[6];
        arr[0] = yaw;
        arr[1] = yawRate;
        arr[2] = pitch;
        arr[3] = pitchRate;
        arr[4] = roll;
        arr[5] = rollRate;
        getLimelightNTTableEntry(limelightName, "botpose_orb").setDoubleArray(arr);
    }

    public static void setRobotOrientation_wpiBlue(String limelightName, double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        double[] arr = new double[6];
        arr[0] = yaw;
        arr[1] = yawRate;
        arr[2] = pitch;
        arr[3] = pitchRate;
        arr[4] = roll;
        arr[5] = rollRate;
        getLimelightNTTableEntry(limelightName, "botpose_wpiblue").setDoubleArray(arr);
    }

    public static void setRobotOrientation_wpiRed(String limelightName, double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        double[] arr = new double[6];
        arr[0] = yaw;
        arr[1] = yawRate;
        arr[2] = pitch;
        arr[3] = pitchRate;
        arr[4] = roll;
        arr[5] = rollRate;
        getLimelightNTTableEntry(limelightName, "botpose_wpired").setDoubleArray(arr);
    }

    public static void setCameraPose_RobotSpace(String limelightName, double camX, double camY, double camZ, double camRoll, double camPitch, double camYaw) {
        double[] arr = new double[6];
        arr[0] = camX;
        arr[1] = camY;
        arr[2] = camZ;
        arr[3] = camRoll;
        arr[4] = camPitch;
        arr[5] = camYaw;
        getLimelightNTTableEntry(limelightName, "camerapose_robotspace_set").setDoubleArray(arr);
    }

    public static NetworkTable getLimelightNTTable(String limelightName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(limelightName));
    }

    public static DoubleArrayEntry getLimelightNTTableEntry(String limelightName, String entryName) {
        return getLimelightNTTable(limelightName).getDoubleArrayTopic(entryName).getEntry(new double[0]);
    }

    public static double getLimelightNTDouble(String limelightName, String entryName) {
        return getLimelightNTTableEntry(limelightName, entryName).getDouble(0.0);
    }

    public static double[] getLimelightNTDoubleArray(String limelightName, String entryName) {
        return getLimelightNTTableEntry(limelightName, entryName).getDoubleArray(new double[0]);
    }

    public static void printLimelightResults(LimelightResults results) {
        // Implement printing if needed
    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public double tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency, double tagCount, double tagSpan, double avgTagDist, double avgTagArea) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
        }
    }

    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        double[] result = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        if (result.length != 11) {
            return null;
        }
        return new PoseEstimate(toPose2D(result), result[5] / 1000.0, result[6] / 1000.0, result[7], result[8], result[9], result[10]);
    }

    // Other getBotPoseEstimate methods similarly

    public static class LimelightTarget_Retro {
        public double tx;
        public double ty;
        public double ta;
        public double thoriz;
        public double tvert;
        public double[] txp;
        public double[] typ;
        public double pipelineLatency;
    }

    public static class LimelightTarget_Fiducial {
        public double fiducialID;
        public String family;
        public double tx;
        public double ty;
        public double ta;
        public double thoriz;
        public double tvert;
        public double[] txp;
        public double[] typ;
        public double pipelineLatency;
        public Pose3d targetToCamera;
        public Pose3d targetToRobot;
        public Pose3d cameraToTarget;
        public Pose3d robotToTarget;
        public Pose3d alternate_targetToRobot;
        public Pose3d alternate_robotToTarget;
    }

    public static class LimelightTarget_Barcode {
        // Implement if needed
    }

    public static class LimelightTarget_Classifier {
        public String className;
        public double classID;
        public double confidence;
        public double tx;
        public double ty;
        public double ta;
        public double[] txp;
        public double[] typ;
        public double pipelineLatency;
    }

    public static class LimelightTarget_Detector {
        public String className;
        public double classID;
        public double confidence;
        public double tx;
        public double ty;
        public double ta;
        public double[] txp;
        public double[] typ;
        public double pipelineLatency;
    }

    public static class Results {
        public boolean valid;
        public List<LimelightTarget_Retro> targetingResults_Retro;
        public List<LimelightTarget_Fiducial> targetingResults_Fiducial;
        public List<LimelightTarget_Classifier> targetingResults_Classifier;
        public List<LimelightTarget_Detector> targetingResults_Detector;
        public List<LimelightTarget_Barcode> targetingResults_Barcode;
        public double latency_pipeline;
        public double latency_capture;
        public double timestamp_LIMELIGHT_publish;
        public double timestamp_RIOFPGA_capture;
        public double pipelineIndex;
        public Pose3d botpose;
        public Pose3d botpose_wpired;
        public Pose3d botpose_wpiblue;
        public Pose3d botpose_targetspace;
        public Pose3d camerapose_targetspace;
        public Pose3d targetpose_cameraspace;
        public Pose3d targetpose_robotspace;
        public Pose3d botpose_orb;
        public Pose3d botpose_orb_wpired;
        public Pose3d botpose_orb_wpiblue;
        public Pose3d camerapose_robotspace;
        public double tl;
        public double cl;
        public double tclass;
        public double jsonDumpLatency;
    }

    public static Results getLatestResults(String limelightName) {
        // Implement JSON parsing using JSONObject or similar
        // For brevity, placeholder
        return new Results();
    }

    // Add other classes and methods as needed from the C++ code
}