package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.networktables.DoubleArrayPublisher;

import static edu.wpi.first.units.Units.*;

public class Telemetry {
    public static Object telemeterize;

    private final double MaxSpeed;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final StructPublisher<Pose2d> drivePose;
    private final StructPublisher<ChassisSpeeds> driveSpeeds;
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates;
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets;
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions;
    private final DoublePublisher driveTimestamp;
    private final DoublePublisher driveOdometryFrequency;
    private final StringPublisher fieldTypePub;
    private final DoubleArrayPublisher fieldPub;

    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[4];
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[4];
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[4];

    private final double[] poseArray = new double[3];
    private final double[] moduleStatesArray = new double[8];
    private final double[] moduleTargetsArray = new double[8];

    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;

        var driveStateTable = inst.getTable("DriveState");
        drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
        driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
        driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
        driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
        driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
        driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
        driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();
        fieldTypePub = driveStateTable.getStringTopic("FieldType").publish();
        fieldPub = driveStateTable.getDoubleArrayTopic("Field").publish();

        for (int i = 0; i < 4; i++) {
            m_moduleMechanisms[i] = new Mechanism2d(1, 1);
            MechanismRoot2d rootSpeed = m_moduleMechanisms[i].getRoot("RootSpeed", 0.5, 0.5);
            m_moduleSpeeds[i] = rootSpeed.append(new MechanismLigament2d("Speed", 0.5, 0, 5, new Color8Bit(Color.kBlue)));

            MechanismRoot2d rootDirection = m_moduleMechanisms[i].getRoot("RootDirection", 0.5, 0.5);
            m_moduleDirections[i] = rootDirection.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite)));
        }

        SignalLogger.start();
    }

    public void telemeterize(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        
        SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);

        fieldTypePub.set("Field2d");
        fieldPub.set(poseArray);

        for (int i = 0; i < 4; i++) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
}