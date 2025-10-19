package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.SignalLogger;

public class Telemetry {
  private final Field2d field = new Field2d();
  private final Mechanism2d[] moduleMechanisms = new Mechanism2d[4];
  private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[4];
  private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[4];

  private static final double MAX_SPEED = 5.0; // adjust based on your drivetrain

  public Telemetry() {
    for (int i = 0; i < 4; i++) {
      moduleMechanisms[i] = new Mechanism2d(2, 2);
      MechanismRoot2d root = moduleMechanisms[i].getRoot("Module" + i, 1, 1);
      moduleDirections[i] = root.append(new MechanismLigament2d("Dir" + i, 0.5, 0));
      moduleSpeeds[i] = root.append(new MechanismLigament2d("Speed" + i, 0.5, 0));
    }

    SmartDashboard.putData("Field", field);
  }

  public void telemeterize(SwerveDriveState state) {
    // --- Write values to the SignalLogger ---
    double[] moduleStatesArray = new double[8];
    double[] moduleTargetsArray = new double[8];

    for (int i = 0; i < 4; i++) {
      moduleStatesArray[i * 2] = state.moduleStates[i].angle.getRadians();
      moduleStatesArray[i * 2 + 1] = state.moduleStates[i].speedMetersPerSecond;
      moduleTargetsArray[i * 2] = state.moduleTargets[i].angle.getRadians();
      moduleTargetsArray[i * 2 + 1] = state.moduleTargets[i].speedMetersPerSecond;
    }

    Pose2d pose = state.pose;

    SignalLogger.writeDoubleArray("DriveState/Pose",
        new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.odometryPeriod);

    // --- Update Field2d visualization ---
    field.setRobotPose(pose);

    // --- Update Mechanism2d modules for dashboard visualization ---
    for (int i = 0; i < 4; i++) {
      double angleDeg = state.moduleStates[i].angle.getDegrees();
      double speedRatio = state.moduleStates[i].speedMetersPerSecond / (2 * MAX_SPEED);

      moduleDirections[i].setAngle(angleDeg);
      moduleSpeeds[i].setAngle(angleDeg);
      moduleSpeeds[i].setLength(speedRatio);

      SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
    }
  }
}
