package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

public class NetworkTables {
    public enum ConstantType {
        Double,
        Int,
        Boolean,
        String,
        Velocity,
        AngularRate,
        Acceleration,
        AngularAcceleration,
        Time,
        Current
    }

    public enum ConstantId {
        MaxSpeed,
        MaxAngularRate,
        ControllerVelocityCurveExponent,
        ControllerRotationCurveExponent,
        ControllerDeadbandPercentage,
        SlewTranslateLimit,
        SlewRotateLimit,
        RollerMovementHoldVelocity,
        RollerMovementForwardVelocity,
        RollerMovementBackwardVelocity,
        RollerMovementCoralEjectVelocity,
        RollerMovementAlgaeIntakeVelocity,
        RollerMovementAlgaeEjectVelocity,
        ArmUpVelocity,
        ArmDownVelocity,
        ArmIntakePosition,
        ArmHoldPosition,
        ArmCoralEjectPosition,
        ArmDefaultPosition,
        ArmMotorForwardNominalPercentOutput,
        ArmMotorReverseNominalPercentOutput,
        ArmMotorForwardPeakPercentOutput,
        ArmMotorReversePeakPercentOutput,
        ArmMotorMagicMotionCruiseVelocity,
        ArmMotorMagicMotionAccelerationVelocity,
        ArmMotorProportionalGainValue,
        ArmMotorIntegralGainValue,
        ArmMotorDerivativeGainValue,
        ArmMotorFeedForwardGainValue,
        ArmSelectedSensorPosition,
        ArmMotorAllowableCloseLoopError,
        ClimbVelocity,
        UnclimbVelocity,
        ClimberTorqueCurrentLimit,
        AutoIntakeAlgaeWait,
        AutoEjectAlgaeWait,
        AutoEjectCoralWait,
        AlgaeIntakeSequenceWait,
        ArmCoralEjectSequenceWait,
        ClimbButton,
        UnclimbButton,
        RollerForwardButton,
        RollerBackwardButton,
        ArmUpButton,
        ArmDownButton,
        ResetEncoderButton,
        AlgaeIntakeButtonAxis,
        AlgaeEjectButtonAxis,
        kNumConstants
    }

    public static class ConstantEntry {
        public String networkTableKey;
        public ConstantType type;

        public static class Value {
            public double doubleValue;
            public int intValue;
            public boolean boolValue;
            public String stringValue;
        }

        public Value defaultValue = new Value();
    }

    private final NetworkTable table;
    private static final String kTableName = "Tuning Constants";
    private static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kMaxAngularRate = 1.2;

    private static final List<ConstantEntry> constantEntries = new ArrayList<>() {
        {
            add(new ConstantEntry() {
                {
                    networkTableKey = "MaxSpeed";
                    type = ConstantType.Velocity;
                    defaultValue.doubleValue = kMaxSpeed; // From C++
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "MaxAngularRate";
                    type = ConstantType.AngularRate;
                    defaultValue.doubleValue = kMaxAngularRate * 2 * Math.PI; // From C++
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ControllerVelocityCurveExponent";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 2.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ControllerRotationCurveExponent";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 2.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ControllerDeadbandPercentage";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.02;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "SlewTranslateLimit";
                    type = ConstantType.Acceleration;
                    defaultValue.doubleValue = 10.0 * kMaxSpeed; // From C++
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "SlewRotateLimit";
                    type = ConstantType.AngularAcceleration;
                    defaultValue.doubleValue = 30.0 * kMaxAngularRate; // From C++
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerMovementHoldVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.05;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerMovementForwardVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.6;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerMovementBackwardVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -0.6;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerMovementCoralEjectVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 1.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerMovementAlgaeIntakeVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 1.0; // Fixed: was 0.5 → now 1.0 (matches C++)
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerMovementAlgaeEjectVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -0.6;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmUpVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.6;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmDownVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -0.2;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmIntakePosition";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -1600;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmHoldPosition";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -500;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmCoralEjectPosition";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -700;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmDefaultPosition";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 700;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorForwardNominalPercentOutput";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorReverseNominalPercentOutput";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorForwardPeakPercentOutput";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.4;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorReversePeakPercentOutput";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -0.4;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorMagicMotionCruiseVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 50.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorMagicMotionAccelerationVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 50.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorProportionalGainValue";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 5.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorIntegralGainValue";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorDerivativeGainValue";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 5.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorFeedForwardGainValue";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.1;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmSelectedSensorPosition";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 0.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmMotorAllowableCloseLoopError";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 5.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ClimbVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = 1.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "UnclimbVelocity";
                    type = ConstantType.Double;
                    defaultValue.doubleValue = -1.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ClimberTorqueCurrentLimit";
                    type = ConstantType.Current;
                    defaultValue.doubleValue = 22.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "AutoIntakeAlgaeWait";
                    type = ConstantType.Time;
                    defaultValue.doubleValue = 1.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "AutoEjectAlgaeWait";
                    type = ConstantType.Time;
                    defaultValue.doubleValue = 1.0;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "AutoEjectCoralWait";
                    type = ConstantType.Time;
                    defaultValue.doubleValue = 0.5;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "AlgaeIntakeSequenceWait";
                    type = ConstantType.Time;
                    defaultValue.doubleValue = 0.01;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmCoralEjectSequenceWait";
                    type = ConstantType.Time;
                    defaultValue.doubleValue = 0.1;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ClimbButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 2;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "UnclimbButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 1;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerForwardButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 4;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "RollerBackwardButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 3;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmUpButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 6;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ArmDownButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 5;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "ResetEncoderButton";
                    type = ConstantType.Int;
                    defaultValue.intValue = 8;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "AlgaeIntakeButtonAxis";
                    type = ConstantType.Int;
                    defaultValue.intValue = 2;
                }
            });
            add(new ConstantEntry() {
                {
                    networkTableKey = "AlgaeEjectButtonAxis";
                    type = ConstantType.Int;
                    defaultValue.intValue = 3;
                }
            });
        }
    };

    public NetworkTables() {
        table = NetworkTableInstance.getDefault().getTable(kTableName);
        SetDefaults();
    }

    private void SetDefaults() {
        for (ConstantEntry entry : constantEntries) {
            switch (entry.type) {
                case Double:
                case Velocity:
                case AngularRate:
                case Acceleration:
                case AngularAcceleration:
                case Time:
                case Current:
                    table.getEntry(entry.networkTableKey).setDefaultDouble(entry.defaultValue.doubleValue);
                    break;
                case Int:
                    table.getEntry(entry.networkTableKey).setDefaultInteger(entry.defaultValue.intValue);
                    break;
                case Boolean:
                    table.getEntry(entry.networkTableKey).setDefaultBoolean(entry.defaultValue.boolValue);
                    break;
                case String:
                    table.getEntry(entry.networkTableKey).setDefaultString(entry.defaultValue.stringValue);
                    break;
            }
        }
    }

    public void RestoreDefaults() {
        for (ConstantEntry entry : constantEntries) {
            switch (entry.type) {
                case Double:
                case Velocity:
                case AngularRate:
                case Acceleration:
                case AngularAcceleration:
                case Time:
                case Current:
                    table.getEntry(entry.networkTableKey).setDouble(entry.defaultValue.doubleValue);
                    break;
                case Int:
                    table.getEntry(entry.networkTableKey).setInteger(entry.defaultValue.intValue);
                    break;
                case Boolean:
                    table.getEntry(entry.networkTableKey).setBoolean(entry.defaultValue.boolValue);
                    break;
                case String:
                    table.getEntry(entry.networkTableKey).setString(entry.defaultValue.stringValue);
                    break;
            }
        }
    }

    public double getDoubleValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Double) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue);
    }

    public int getIntValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Int) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return (int) table.getEntry(entry.networkTableKey).getInteger(entry.defaultValue.intValue);
    }

    public boolean getBooleanValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Boolean) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return table.getEntry(entry.networkTableKey).getBoolean(entry.defaultValue.boolValue);
    }

    public String getStringValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.String) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return table.getEntry(entry.networkTableKey).getString(entry.defaultValue.stringValue);
    }

    public Measure<AngularVelocityUnit> getAngularRateValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.AngularRate) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return RadiansPerSecond.of(table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue));
    }

    public LinearVelocity getVelocityValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Velocity) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return MetersPerSecond.of(table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue));
    }

    public LinearAcceleration getAccelerationValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Acceleration) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return MetersPerSecondPerSecond
                .of(table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue));
    }

    public Measure<AngularAccelerationUnit> getAngularAccelerationValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.AngularAcceleration) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return RadiansPerSecondPerSecond
                .of(table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue));
    }

    public Measure<TimeUnit> getTimeValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Time) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return Seconds.of(table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue));
    }

    public Measure<CurrentUnit> getCurrentValue(ConstantId id) {
        ConstantEntry entry = constantEntries.get(id.ordinal());
        if (entry.type != ConstantType.Current) {
            throw new RuntimeException("Constant type mismatch for " + entry.networkTableKey);
        }
        return Amps.of(table.getEntry(entry.networkTableKey).getDouble(entry.defaultValue.doubleValue));
    }
}