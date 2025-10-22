package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * LimelightHelpers v1.6.0 (Java)
 * 
 * A utility class for interfacing with Limelight vision systems in FRC robots.
 * Based on the official LimelightHelpers library.
 * 
 * This is a single-file library - just copy this file into your robot package.
 */
public class LimelightHelpers {

    /**
     * Sanitizes the Limelight name. Returns "limelight" if name is empty or null.
     */
    public static String sanitizeName(String name) {
        if (name == null || name.isEmpty()) {
            return "limelight";
        }
        return name;
    }

    /**
     * Gets the NetworkTable for the specified Limelight.
     */
    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    /**
     * Gets a specific NetworkTableEntry from the Limelight table.
     */
    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    /**
     * Gets a double value from the Limelight NetworkTable.
     */
    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    /**
     * Gets a double array from the Limelight NetworkTable.
     */
    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    /**
     * Gets a string from the Limelight NetworkTable.
     */
    public static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    /**
     * Sets a double value in the Limelight NetworkTable.
     */
    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    /**
     * Sets a double array in the Limelight NetworkTable.
     */
    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] vals) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(vals);
    }

    // ========== BASIC TARGETING DATA ==========

    /**
     * Gets the horizontal offset from crosshair to target (degrees).
     */
    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    /**
     * Gets whether the Limelight has a valid target (0 or 1).
     */
    public static double getTV(String limelightName) {
        return getLimelightNTDouble(limelightName, "tv");
    }

    /**
     * Gets the vertical offset from crosshair to target (degrees).
     */
    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    /**
     * Gets the target area (0% to 100% of image).
     */
    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    /**
     * Gets the pipeline's latency contribution (ms). Add to "cl" for total latency.
     */
    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    /**
     * Gets the capture pipeline latency (ms).
     */
    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    /**
     * Gets the JSON dump of targeting results.
     */
    public static String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    // ========== POSE ESTIMATION ==========

    /**
     * Converts a double array to Pose3d.
     */
    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(
                Math.toRadians(inData[3]),
                Math.toRadians(inData[4]),
                Math.toRadians(inData[5])
            )
        );
    }

    /**
     * Converts a double array to Pose2d.
     */
    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
            new Translation2d(inData[0], inData[1]),
            Rotation2d.fromDegrees(inData[5])
        );
    }

    /**
     * Gets the robot's position in field space (meters).
     */
    public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Gets the robot's position in field space (blue alliance origin).
     */
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    /**
     * Gets the robot's position in field space (red alliance origin).
     */
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Gets the robot's position relative to the primary AprilTag.
     */
    public static double[] getBotpose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    /**
     * Gets the camera's position relative to the primary AprilTag.
     */
    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    /**
     * Gets the camera's position relative to the robot.
     */
    public static double[] getCameraPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
    }

    /**
     * Gets the target's position relative to the camera.
     */
    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    /**
     * Gets the target's position relative to the robot.
     */
    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    /**
     * Gets the average color of the target (HSV).
     */
    public static double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    /**
     * Gets the ID of the primary AprilTag.
     */
    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    /**
     * Gets the class ID of the detected object.
     */
    public static String getNeuralClassID(String limelightName) {
        return getLimelightNTString(limelightName, "tclass");
    }

    // ========== CONFIGURATION ==========

    /**
     * Sets the active pipeline index (0-9).
     */
    public static void setPipelineIndex(String limelightName, int index) {
        setLimelightNTDouble(limelightName, "pipeline", index);
    }

    /**
     * Sets the priority tag ID for multi-tag detection.
     */
    public static void setPriorityTagID(String limelightName, int ID) {
        setLimelightNTDouble(limelightName, "priorityid", ID);
    }

    /**
     * Sets LED mode to pipeline control.
     */
    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    /**
     * Forces LEDs off.
     */
    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    /**
     * Forces LEDs to blink.
     */
    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    /**
     * Forces LEDs on.
     */
    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    /**
     * Sets stream mode to standard (side-by-side if two cameras).
     */
    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    /**
     * Sets stream mode to PiP Main (secondary camera in primary).
     */
    public static void setStreamMode_PiPMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    /**
     * Sets stream mode to PiP Secondary (primary camera in secondary).
     */
    public static void setStreamMode_PiPSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    /**
     * Sets the crop window for the pipeline. 
     * The crop window in the UI must be completely open for dynamic cropping to work.
     */
    public static void setCropWindow(String limelightName, double cropXMin, double cropXMax, 
                                    double cropYMin, double cropYMax) {
        double[] cropWindow = {cropXMin, cropXMax, cropYMin, cropYMax};
        setLimelightNTDoubleArray(limelightName, "crop", cropWindow);
    }

    /**
     * Sets the robot's orientation for MegaTag2.
     * 
     * @param yaw Yaw in degrees
     * @param yawRate Yaw rate in degrees/sec
     * @param pitch Pitch in degrees
     * @param pitchRate Pitch rate in degrees/sec
     * @param roll Roll in degrees
     * @param rollRate Roll rate in degrees/sec
     */
    public static void setRobotOrientation(String limelightName, double yaw, double yawRate,
                                          double pitch, double pitchRate, double roll, double rollRate) {
        double[] entries = {yaw, yawRate, pitch, pitchRate, roll, rollRate};
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
    }

    /**
     * Sets fiducial downscaling for performance.
     * 
     * @param downscale Valid values: 1.0, 1.5, 2.0, 3.0, 4.0
     */
    public static void setFiducialDownscaling(String limelightName, double downscale) {
        int d = 1; // Default to 1.0
        if (downscale == 1.5) d = 2;
        else if (downscale == 2.0) d = 3;
        else if (downscale == 3.0) d = 4;
        else if (downscale == 4.0) d = 5;
        setLimelightNTDouble(limelightName, "fiducial_downscale_set", d);
    }

    /**
     * Sets which AprilTag IDs the pipeline should look for.
     */
    public static void setFiducialIDFiltersOverride(String limelightName, int[] validIDs) {
        double[] validIDsDouble = new double[validIDs.length];
        for (int i = 0; i < validIDs.length; i++) {
            validIDsDouble[i] = validIDs[i];
        }
        setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
    }

    /**
     * Sets the camera pose in robot space. 
     * The UI camera pose must be set to zeros for this to work.
     */
    public static void setCameraPose_RobotSpace(String limelightName, double forward, double side,
                                               double up, double roll, double pitch, double yaw) {
        double[] entries = {forward, side, up, roll, pitch, yaw};
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    /**
     * Sets data to be sent to Python scripts.
     */
    public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    /**
     * Gets data from Python scripts.
     */
    public static double[] getPythonScriptData(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    // ========== HELPER METHODS ==========

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0.0;
        }
        return inData[position];
    }

    // ========== RAW FIDUCIALS ==========

    /**
     * Represents a raw fiducial detection.
     */
    public static class RawFiducial {
        public int id;
        public double txnc;
        public double tync;
        public double ta;
        public double distToCamera;
        public double distToRobot;
        public double ambiguity;

        public RawFiducial(int id, double txnc, double tync, double ta,
                          double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    /**
     * Gets raw fiducial data from the Limelight.
     */
    public static List<RawFiducial> getRawFiducials(String limelightName) {
        double[] rawFiducialArray = getLimelightNTDoubleArray(limelightName, "rawfiducials");
        int valsPerEntry = 7;
        
        if (rawFiducialArray.length % valsPerEntry != 0) {
            return new ArrayList<>();
        }

        int numFiducials = rawFiducialArray.length / valsPerEntry;
        List<RawFiducial> rawFiducials = new ArrayList<>();

        for (int i = 0; i < numFiducials; i++) {
            int baseIndex = i * valsPerEntry;
            int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
            double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);

            rawFiducials.add(new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity));
        }

        return rawFiducials;
    }

    // ========== RAW DETECTIONS ==========

    /**
     * Represents a raw neural network detection.
     */
    public static class RawDetection {
        public int classId;
        public double txnc;
        public double tync;
        public double ta;
        public double corner0_X;
        public double corner0_Y;
        public double corner1_X;
        public double corner1_Y;
        public double corner2_X;
        public double corner2_Y;
        public double corner3_X;
        public double corner3_Y;

        public RawDetection(int classId, double txnc, double tync, double ta,
                          double corner0_X, double corner0_Y, double corner1_X, double corner1_Y,
                          double corner2_X, double corner2_Y, double corner3_X, double corner3_Y) {
            this.classId = classId;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.corner0_X = corner0_X;
            this.corner0_Y = corner0_Y;
            this.corner1_X = corner1_X;
            this.corner1_Y = corner1_Y;
            this.corner2_X = corner2_X;
            this.corner2_Y = corner2_Y;
            this.corner3_X = corner3_X;
            this.corner3_Y = corner3_Y;
        }
    }

    /**
     * Gets raw detection data from the Limelight.
     */
    public static List<RawDetection> getRawDetections(String limelightName) {
        double[] rawDetectionArray = getLimelightNTDoubleArray(limelightName, "rawdetections");
        int valsPerEntry = 12;

        if (rawDetectionArray.length % valsPerEntry != 0) {
            return new ArrayList<>();
        }

        int numDetections = rawDetectionArray.length / valsPerEntry;
        List<RawDetection> rawDetections = new ArrayList<>();

        for (int i = 0; i < numDetections; i++) {
            int baseIndex = i * valsPerEntry;
            int classId = (int) extractArrayEntry(rawDetectionArray, baseIndex);
            double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            double ta = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);

            rawDetections.add(new RawDetection(classId, txnc, tync, ta,
                corner0_X, corner0_Y, corner1_X, corner1_Y,
                corner2_X, corner2_Y, corner3_X, corner3_Y));
        }

        return rawDetections;
    }

    // ========== POSE ESTIMATE ==========

    /**
     * Represents a complete pose estimate from the Limelight.
     */
    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;
        public List<RawFiducial> rawFiducials;

        public PoseEstimate() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0.0;
            this.latency = 0.0;
            this.tagCount = 0;
            this.tagSpan = 0.0;
            this.avgTagDist = 0.0;
            this.avgTagArea = 0.0;
            this.rawFiducials = new ArrayList<>();
        }

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                          int tagCount, double tagSpan, double avgTagDist, double avgTagArea,
                          List<RawFiducial> rawFiducials) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
        }
    }

    /**
     * Gets a pose estimate from the specified NetworkTable entry.
     */
    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName) {
        NetworkTableEntry poseEntry = getLimelightNTTableEntry(limelightName, entryName);
        double[] poseArray = poseEntry.getDoubleArray(new double[0]);
        
        if (poseArray.length == 0) {
            return null;
        }

        Pose2d pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);

        // Calculate timestamp: getLastChange is in microseconds, latency is in milliseconds
        double timestamp = (poseEntry.getLastChange() / 1000000.0) - (latency / 1000.0);

        List<RawFiducial> rawFiducials = new ArrayList<>();
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length == expectedTotalVals) {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) extractArrayEntry(poseArray, baseIndex);
                double txnc = extractArrayEntry(poseArray, baseIndex + 1);
                double tync = extractArrayEntry(poseArray, baseIndex + 2);
                double ta = extractArrayEntry(poseArray, baseIndex + 3);
                double distToCamera = extractArrayEntry(poseArray, baseIndex + 4);
                double distToRobot = extractArrayEntry(poseArray, baseIndex + 5);
                double ambiguity = extractArrayEntry(poseArray, baseIndex + 6);
                rawFiducials.add(new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity));
            }
        }

        return new PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

    /**
     * Gets the robot pose estimate in WPILib blue alliance coordinates.
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue");
    }

    /**
     * Gets the robot pose estimate in WPILib red alliance coordinates.
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired");
    }

    /**
     * Gets the robot pose estimate using MegaTag2 in blue alliance coordinates.
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue");
    }

    /**
     * Gets the robot pose estimate using MegaTag2 in red alliance coordinates.
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired");
    }

    // ========== JSON RESULTS (Advanced Usage) ==========
    // Note: This requires Jackson for JSON parsing
    // Add to build.gradle: implementation 'com.fasterxml.jackson.core:jackson-databind:2.15.0'

    private static final ObjectMapper mapper = new ObjectMapper()
        .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    /**
     * Results from JSON parsing - contains all targeting data.
     */
    public static class LimelightResults {
        @JsonProperty("Results")
        public Results targetingResults;

        public LimelightResults() {
            this.targetingResults = new Results();
        }
    }

    public static class Results {
        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        public int valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("Retro")
        public List<RetroreflectiveResult> retroResults;

        @JsonProperty("Fiducial")
        public List<FiducialResult> fiducialResults;

        @JsonProperty("Detector")
        public List<DetectorResult> detectorResults;

        @JsonProperty("Classifier")
        public List<ClassifierResult> classifierResults;

        public Results() {
            this.retroResults = new ArrayList<>();
            this.fiducialResults = new ArrayList<>();
            this.detectorResults = new ArrayList<>();
            this.classifierResults = new ArrayList<>();
        }
    }

    public static class RetroreflectiveResult {
        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("pts")
        public double[][] corners;
    }

    public static class FiducialResult {
        @JsonProperty("fID")
        public int fiducialID;

        @JsonProperty("fam")
        public String family;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("t6c_ts")
        public double[] cameraPose_TargetSpace;

        @JsonProperty("t6t_cs")
        public double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        public double[] targetPose_RobotSpace;

        @JsonProperty("t6r_fs")
        public double[] robotPose_FieldSpace;

        @JsonProperty("pts")
        public double[][] corners;
    }

    public static class DetectorResult {
        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public int classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("pts")
        public double[][] corners;
    }

    public static class ClassifierResult {
        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public int classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("ta")
        public double ta;
    }

    /**
     * Gets the latest targeting results by parsing the JSON dump.
     * This provides access to all detection data in a structured format.
     * 
     * @param limelightName Name of the Limelight
     * @return LimelightResults object containing all targeting data
     */
    public static LimelightResults getLatestResults(String limelightName) {
        long start = System.nanoTime();
        String jsonString = getJSONDump(limelightName);
        
        try {
            LimelightResults results = mapper.readValue(jsonString, LimelightResults.class);
            long end = System.nanoTime();
            double latencyMs = (end - start) / 1_000_000.0;
            
            // Store JSON parsing latency
            if (results.targetingResults != null) {
                // You can add a custom field to store this if needed
                System.out.println("JSON parse latency: " + latencyMs + " ms");
            }
            
            return results;
        } catch (JsonProcessingException e) {
            System.err.println("Error parsing Limelight JSON: " + e.getMessage());
            return new LimelightResults();
        }
    }

    /**
     * Prints a summary of the Limelight results to the console.
     * Useful for debugging.
     */
    public static void printResults(LimelightResults results) {
        if (results == null || results.targetingResults == null) {
            System.out.println("No results available");
            return;
        }

        Results r = results.targetingResults;
        System.out.println("=== Limelight Results ===");
        System.out.println("Pipeline: " + r.pipelineID);
        System.out.println("Valid: " + (r.valid == 1 ? "Yes" : "No"));
        System.out.println("Latency: " + r.latency_pipeline + " ms");
        
        if (r.botpose_wpiblue != null && r.botpose_wpiblue.length >= 6) {
            Pose2d pose = toPose2D(r.botpose_wpiblue);
            System.out.println("Robot Pose (Blue): " + pose.toString());
        }

        System.out.println("Fiducials detected: " + r.fiducialResults.size());
        for (FiducialResult fid : r.fiducialResults) {
            System.out.println("  - ID: " + fid.fiducialID + ", TX: " + fid.tx + ", TY: " + fid.ty);
        }

        System.out.println("Detections: " + r.detectorResults.size());
        for (DetectorResult det : r.detectorResults) {
            System.out.println("  - Class: " + det.className + ", Confidence: " + det.confidence);
        }
    }

    // ========== UTILITY METHODS FOR VISION PROCESSING ==========

    /**
     * Checks if the Limelight has any valid targets.
     */
    public static boolean hasTarget(String limelightName) {
        return getTV(limelightName) == 1.0;
    }

    /**
     * Gets the pipeline index currently running.
     */
    public static int getCurrentPipelineIndex(String limelightName) {
        return (int) getLimelightNTDouble(limelightName, "getpipe");
    }

    /**
     * Gets the horizontal offset with fallback if no target.
     */
    public static double getTXSafe(String limelightName, double defaultValue) {
        if (!hasTarget(limelightName)) {
            return defaultValue;
        }
        return getTX(limelightName);
    }

    /**
     * Gets the vertical offset with fallback if no target.
     */
    public static double getTYSafe(String limelightName, double defaultValue) {
        if (!hasTarget(limelightName)) {
            return defaultValue;
        }
        return getTY(limelightName);
    }

    /**
     * Gets the target area with fallback if no target.
     */
    public static double getTASafe(String limelightName, double defaultValue) {
        if (!hasTarget(limelightName)) {
            return defaultValue;
        }
        return getTA(limelightName);
    }

    /**
     * Calculates the distance to target using pinhole camera model.
     * 
     * @param targetHeightMeters Height of the target above ground
     * @param cameraHeightMeters Height of the camera above ground
     * @param cameraPitchRadians Camera pitch angle in radians (positive = up)
     * @param targetPitchRadians Target pitch from camera in radians
     * @return Distance to target in meters
     */
    public static double calculateDistanceToTarget(double targetHeightMeters, 
                                                   double cameraHeightMeters,
                                                   double cameraPitchRadians, 
                                                   double targetPitchRadians) {
        double heightDifference = targetHeightMeters - cameraHeightMeters;
        double angleToTarget = cameraPitchRadians + targetPitchRadians;
        return heightDifference / Math.tan(angleToTarget);
    }

    /**
     * Calculates the horizontal distance to target using the target's vertical offset.
     * 
     * @param limelightName Name of the Limelight
     * @param targetHeightMeters Height of the target above ground
     * @param cameraHeightMeters Height of the camera above ground  
     * @param cameraPitchDegrees Camera pitch angle in degrees (positive = up)
     * @return Distance to target in meters, or -1 if no target
     */
    public static double getDistanceToTarget(String limelightName, 
                                            double targetHeightMeters,
                                            double cameraHeightMeters, 
                                            double cameraPitchDegrees) {
        if (!hasTarget(limelightName)) {
            return -1.0;
        }

        double ty = getTY(limelightName);
        double cameraPitchRadians = Math.toRadians(cameraPitchDegrees);
        double targetPitchRadians = Math.toRadians(ty);

        return calculateDistanceToTarget(targetHeightMeters, cameraHeightMeters, 
                                        cameraPitchRadians, targetPitchRadians);
    }

    /**
     * Takes a snapshot and saves it to the Limelight.
     * Useful for debugging or logging.
     */
    public static void takeSnapshot(String limelightName) {
        setLimelightNTDouble(limelightName, "snapshot", 1.0);
    }

    /**
     * Resets the snapshot flag.
     */
    public static void resetSnapshot(String limelightName) {
        setLimelightNTDouble(limelightName, "snapshot", 0.0);
    }

    // ========== EXAMPLE USAGE ==========

    /**
     * Example method showing how to use LimelightHelpers for vision alignment.
     * This is just a template - customize for your robot!
     */
    public static class UsageExamples {
        
        /**
         * Example: Get pose estimate and check quality
         */
        public static void examplePoseEstimation() {
            String limelightName = "limelight";
            
            PoseEstimate poseEstimate = getBotPoseEstimate_wpiBlue(limelightName);
            
            if (poseEstimate != null && poseEstimate.tagCount > 0) {
                System.out.println("Robot pose: " + poseEstimate.pose);
                System.out.println("Tags seen: " + poseEstimate.tagCount);
                System.out.println("Average tag distance: " + poseEstimate.avgTagDist + " m");
                
                // Check if pose is reliable (multiple tags, not too far away)
                boolean isReliable = poseEstimate.tagCount >= 2 && 
                                    poseEstimate.avgTagDist < 5.0 &&
                                    poseEstimate.avgTagArea > 0.1;
                
                if (isReliable) {
                    System.out.println("Pose estimate is reliable!");
                    // Use this pose to update odometry
                }
            }
        }

        /**
         * Example: Check for specific AprilTag
         */
        public static void exampleFiducialDetection() {
            String limelightName = "limelight";
            
            List<RawFiducial> fiducials = getRawFiducials(limelightName);
            
            // Look for a specific tag (e.g., ID 4)
            for (RawFiducial fid : fiducials) {
                if (fid.id == 4) {
                    System.out.println("Found tag 4!");
                    System.out.println("Distance: " + fid.distToRobot + " m");
                    System.out.println("Ambiguity: " + fid.ambiguity);
                    
                    // Low ambiguity = good detection
                    if (fid.ambiguity < 0.2) {
                        System.out.println("High confidence detection!");
                    }
                }
            }
        }

        /**
         * Example: Configure Limelight for different game modes
         */
        public static void exampleConfiguration() {
            String limelightName = "limelight";
            
            // Autonomous: Use AprilTags with LEDs off
            setPipelineIndex(limelightName, 0);
            setLEDMode_ForceOff(limelightName);
            
            // Teleop: Use retroreflective targets with LEDs on
            setPipelineIndex(limelightName, 1);
            setLEDMode_ForceOn(limelightName);
            
            // Driver mode: Turn off processing to save CPU
            setPipelineIndex(limelightName, 9); // Assume pipeline 9 is driver camera
            setLEDMode_ForceOff(limelightName);
        }

        /**
         * Example: Basic targeting for simple aiming
         */
        public static void exampleSimpleTargeting() {
            String limelightName = "limelight";
            
            if (hasTarget(limelightName)) {
                double tx = getTX(limelightName);
                double ty = getTY(limelightName);
                double area = getTA(limelightName);
                
                System.out.println("Target found!");
                System.out.println("Horizontal offset: " + tx + " degrees");
                System.out.println("Vertical offset: " + ty + " degrees");
                System.out.println("Area: " + area + "%");
                
                // Use tx to aim the robot
                // Use ty or area to determine distance
            } else {
                System.out.println("No target found");
            }
        }

        /**
         * Example: Using JSON results for complete data
         */
        public static void exampleJSONResults() {
            String limelightName = "limelight";
            
            LimelightResults results = getLatestResults(limelightName);
            
            if (results.targetingResults.valid == 1) {
                // Access all detections
                for (FiducialResult fid : results.targetingResults.fiducialResults) {
                    System.out.println("Fiducial ID " + fid.fiducialID + 
                                     " at (" + fid.tx + ", " + fid.ty + ")");
                }
                
                for (DetectorResult det : results.targetingResults.detectorResults) {
                    System.out.println("Detected " + det.className + 
                                     " with " + (det.confidence * 100) + "% confidence");
                }
            }
            
            printResults(results);
        }
    }
}