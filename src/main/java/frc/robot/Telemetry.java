package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.units.collections.*;
import edu.wpi.first.units.mutable.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class Telemetry {
    private final Measure<VelocityUnit<DistanceUnit>> MaxSpeed;

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

    public Telemetry(Measure<VelocityUnit<DistanceUnit>> maxSpeed) {
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

    public void Telemeterize(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp.in(Seconds));
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod.in(Seconds));

        SignalLogger.writeDoubleArray("DriveState/Pose", new double[] {state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()});
        // Write module states and targets similarly

        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()});

        for (int i = 0; i < 4; i++) {
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed.in(MetersPerSecond)));
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
}