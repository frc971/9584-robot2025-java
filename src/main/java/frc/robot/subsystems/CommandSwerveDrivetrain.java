package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private SwerveRequest.SysIdSwerveTranslation m_sysIdRoutineToApply = m_translationCharacterization;

    // SysId routines as before...

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double odometryUpdateFrequency, double[] odometryStandardDeviation, double[] visionStandardDeviation, SwerveModuleConstants... modules) {
        super(driveTrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        configureAutoBuilder();
        if (com.ctre.phoenix6.Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configureAutoBuilder() {
        // As before...
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        // As before...
    }

    public Command applyRequest(java.util.function.Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> setControl(requestSupplier.get()), this);
    }

    public Command sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }
}