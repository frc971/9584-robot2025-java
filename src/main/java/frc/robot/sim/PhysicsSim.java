package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MomentOfInertia;

import java.util.ArrayList;
import java.util.List;

public class PhysicsSim {
    private static final PhysicsSim instance = new PhysicsSim();
    private final List<SimProfile> simProfiles = new ArrayList<>();

    private PhysicsSim() {}

    public static PhysicsSim getInstance() {
        return instance;
    }

    public void addTalonFX(TalonFX talonFX, Measure<MomentOfInertia> rotorInertia) {
        simProfiles.add(new TalonFXSimProfile(talonFX, rotorInertia));
    }

    public void run() {
        for (SimProfile simProfile : simProfiles) {
            simProfile.run();
        }
    }
}