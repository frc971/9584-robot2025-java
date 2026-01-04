package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;
import frc.robot.utils.simulation.SimSwerveConstants;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem  {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private static double m_lastSimTime;
    
    private final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, 
            SwerveModuleConstants<?, ?, ?>... modules) {
            super(drivetrainConstants, 
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }


    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Red ? 
                        kRedAlliancePerspectiveRotation : 
                        kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        if (mapleSimSwerveDrivetrain != null) {
            Pose2d simPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
            super.resetPose(simPose);
            Logger.recordOutput("Drive/Pose", simPose);
          } else {
            Logger.recordOutput("Drive/Pose", getState().Pose);
          }
      
          Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
          Logger.recordOutput("Drive/TargetStates", getState().ModuleTargets);
          Logger.recordOutput("Drive/MeasuredStates", getState().ModuleStates);
          Logger.recordOutput("Drive/MeasuredSpeeds", getState().Speeds);
        }

    
    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
        mapleSimSwerveDrivetrain =
        new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            SimSwerveConstants.ROBOT_MASS,
            SimSwerveConstants.BUMPER_LENGTH_X,
            SimSwerveConstants.BUMPER_LENGTH_Y,
            SimSwerveConstants.DRIVE_MOTOR_WHEEL,
            SimSwerveConstants.STEER_MOTOR_WHEEL,
            SimSwerveConstants.WHEEL_COF,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
    
    // Initialize simulation pose to inside the field on black line for red alliance
    double redAllianceInitialSimX = 10.2;
    int redAllianceInitialSimY = 4;

    Pose2d initialSimPose = new Pose2d(redAllianceInitialSimX, redAllianceInitialSimY, new Rotation2d(0));
    mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(initialSimPose);
    }

  @Override
  public void resetPose(Pose2d pose) {
    if (this.mapleSimSwerveDrivetrain != null)
      mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
    Timer.delay(0.1); // wait for simulation to update
    super.resetPose(pose);
  }

    public Command applyRequest(java.util.function.Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> setControl(requestSupplier.get()), this);
    }

    @FunctionalInterface
    public interface DeviceConstructor {
        BaseStatusSignal create(int deviceId, String canbus);
    }

    /**
     * Injects vision data into the CTRE internal pose estimator.
     * * @param pose The robot's pose as seen by the Limelight.
     * @param timestamp The timestamp of the measurement in seconds.
     * @param stdDevs The trust level (Standard Deviations) for this measurement.
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        // super.addVisionMeasurement is the correct Phoenix 6 API
        super.addVisionMeasurement(pose, timestamp, stdDevs);
    }
}