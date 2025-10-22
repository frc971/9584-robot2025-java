package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_container = new RobotContainer();

    private static final boolean kUseLimelight = false; // Set to false to disable limelight for now

    private double m_autonomousStartTime = -1.0;

    @Override
    public void testPeriodic() {}
    
    public void robotInit() {
        m_container.robotInit();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Removed getCANUsagePercent() as it doesn't exist in newer WPILib
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
        SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        SmartDashboard.putNumber("Code Runtime (ms)", Timer.getFPGATimestamp() * 1000.0);

        if (kUseLimelight) {
            var driveState = m_container.drivetrain.getState();
            var heading = driveState.Pose.getRotation().getDegrees();
            var omega = driveState.Speeds.omegaRadiansPerSecond;

            // NOTE: You need to add LimelightHelpers.java to your project
            // Download from: https://github.com/LimelightVision/limelightlib-wpijava
            /*
            for (String limelightName : Constants.LimelightConstants.limelightNames) {
                LimelightHelpers.setRobotOrientation(limelightName, heading, 0, 0, 0, 0, 0);
                var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                double currentTime = Timer.getFPGATimestamp() * 1000.0;
                if (m_autonomousStartTime < 0 || currentTime - m_autonomousStartTime < 200.0) {
                    // Skip
                } else {
                    if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omega) < 2) {
                        m_container.drivetrain.addVisionMeasurement(
                            llMeasurement.pose,
                            llMeasurement.timestampSeconds,
                            new double[]{llMeasurement.avgTagDist * 5, llMeasurement.avgTagDist * 5, llMeasurement.avgTagDist * 5}
                        );
                    }
                }
            }
            */
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_autonomousStartTime = Timer.getFPGATimestamp() * 1000.0;
        m_autonomousCommand = m_container.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        m_container.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_container.teleopInit();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}