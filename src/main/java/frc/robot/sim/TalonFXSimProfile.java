package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MomentOfInertia;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.*;

public class TalonFXSimProfile extends SimProfile {
    private final DCMotorSim motorSim;
    private final TalonFXSimState talonFXSim;

    private static final double kMotorResistance = 0.002;

    public TalonFXSimProfile(TalonFX talonFX, Measure<MomentOfInertia> rotorInertia) {
        motorSim = new DCMotorSim(
            LinearSystemId.identifyDrivetrainSystem(DCMotor.getKrakenX60(1), rotorInertia.in(KilogramsSquareMeters), 1),
            DCMotor.getKrakenX60(1),
            1
        );
        talonFXSim = talonFX.getSimState();
    }

    @Override
    public void run() {
        motorSim.setInputVoltage(talonFXSim.getMotorVoltage());

        motorSim.update(getPeriod().in(Seconds));

        talonFXSim.setRawRotorPosition(motorSim.getAngularPosition().getRadians());
        talonFXSim.setRotorVelocity(motorSim.getAngularVelocityRadPerSec() / (2 * Math.PI));

        talonFXSim.setSupplyVoltage(Volts.of(12 - talonFXSim.getSupplyCurrent() * kMotorResistance));
    }
}