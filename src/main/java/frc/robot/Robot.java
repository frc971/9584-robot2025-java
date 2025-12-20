package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Robot extends TimedRobot {

    private static final double kLowBatteryVoltage = 11.8;
    private static final double kLowBatteryDisabledTime = 1.5; //seconds

    private final Timer batteryTimer = new Timer();
    private boolean lowBatteryAlert = false;


    private Command m_autonomousCommand;

    private final RobotContainer m_container = new RobotContainer();

    private static final boolean kUseLimelight = true; // Set to true to enable limelight for now

    private double m_autonomousStartTime = -1.0;

    private void configureLimelights() {
        for (String limelightName : Constants.LimelightConstants.limelightNames) {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            LimelightHelpers.setPipelineIndex(limelightName, 0);
            System.out.println("Configured Limelight: " + limelightName);
        }
    }
    
    private void updateLimelightTelemetry() {
        for (String limelightName : Constants.LimelightConstants.limelightNames) {
            boolean hasTarget = LimelightHelpers.hasTarget(limelightName);
            SmartDashboard.putBoolean(limelightName + "/HasTarget", hasTarget);
            
            if (hasTarget) {
                SmartDashboard.putNumber(limelightName + "/TX", 
                    LimelightHelpers.getTX(limelightName));
                SmartDashboard.putNumber(limelightName + "/TY", 
                    LimelightHelpers.getTY(limelightName));
                SmartDashboard.putNumber(limelightName + "/TA", 
                    LimelightHelpers.getTA(limelightName));
                SmartDashboard.putNumber(limelightName + "/FiducialID", 
                    LimelightHelpers.getFiducialID(limelightName));
            }
            
            SmartDashboard.putNumber(limelightName + "/Pipeline", 
                LimelightHelpers.getCurrentPipelineIndex(limelightName));
            SmartDashboard.putNumber(limelightName + "/Latency", 
                LimelightHelpers.getLatency_Pipeline(limelightName) + 
                LimelightHelpers.getLatency_Capture(limelightName));
        }
    }
    
    private void updateVisionMeasurement() {
        var driveState = m_container.drivetrain.getState();
        var heading = driveState.Pose.getRotation().getDegrees();
        var omega = driveState.Speeds.omegaRadiansPerSecond;
    
        for (String limelightName : Constants.LimelightConstants.limelightNames) {
            LimelightHelpers.setRobotOrientation(
                limelightName, 
                heading, 
                omega * 180.0 / Math.PI,
                0, 0, 0, 0
            );
            
            PoseEstimate llMeasurement = null;
            var alliance = DriverStation.getAlliance();
            
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                } else {
                    llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
                }
            } else {
                llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            }
            
            double currentTime = Timer.getFPGATimestamp() * 1000.0;
            
            if (m_autonomousStartTime < 0 || currentTime - m_autonomousStartTime < 200.0) {
                continue;
            }
            
            if (llMeasurement != null && 
                llMeasurement.tagCount > 0 && 
                Math.abs(omega) < 2.0) {
                
                double xyStdDev = llMeasurement.avgTagDist * 0.5;
                double rotStdDev = llMeasurement.avgTagDist * 0.5;
                
                if (llMeasurement.tagCount >= 2) {
                    xyStdDev *= 0.5;
                    rotStdDev *= 0.5;
                }
                
                SmartDashboard.putNumber(limelightName + "/VisionTagCount", llMeasurement.tagCount);
                SmartDashboard.putNumber(limelightName + "/VisionAvgDist", llMeasurement.avgTagDist);
                
                m_container.drivetrain.addVisionMeasurement(
                    llMeasurement.pose,
                    llMeasurement.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
                );
                //TODO: Not sure if this is working or no
            }
        }
    }

    @Override
    public void testPeriodic() {}
    
    public void robotInit() {
        m_container.robotInit();

        if (kUseLimelight) {
            configureLimelights();
        }

        //simulation joystick warning suppression
        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        disabledTimer.start();

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

        //TODO: Vision updates
        if (kUseLimelight) {
            updateLimelightTelemetry();
            updateVisionMeasurement();

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

        if(DriverStation.isEnabled()){
            batteryTimer.reset();
        }
        double batteryVoltage = RobotController.getBatteryVoltage();

        if(batteryVoltage<= kLowBatteryVoltage && batteryTimer.hasElapsed(kLowBatteryDisabledTime)){
            lowBatteryAlert = true;
        } else {
            lowBatteryAlert = false;
        }

        SmartDashboard.putBoolean("LowBattery", lowBatteryAlert);
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
        
        if (kUseLimelight) {
            for (String limelightName : Constants.LimelightConstants.limelightNames) {
                LimelightHelpers.setPipelineIndex(limelightName, 0);
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            }
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_container.teleopInit();
        
        if (kUseLimelight) {
            for (String limelightName : Constants.LimelightConstants.limelightNames) {
                LimelightHelpers.setPipelineIndex(limelightName, 0);
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            }
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        // This prevents the default message and gives you control
        // If you have physics simulation, put it here
        // For now: do nothing fast
    }
}
