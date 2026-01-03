package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.VisionSubsystem;

public class VisionTestHarness {
    public static void main(String[] args) {
        edu.wpi.first.hal.HAL.initialize(500, 0);

        // Start as the Server so the Limelight Client can connect
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.stopClient();
        nt.startServer();

        // Instantiate the subsystem
        VisionSubsystem vision = new VisionSubsystem();

        System.out.println("--- Server Started: Waiting for Limelight 10.95.84.11 ---");

        while (true) {
            if (nt.getConnections().length > 0) {
                if (vision.hasTarget()) {
                    System.out.printf("[LOG] Seeing: %s | TX: %.2f | TY: %.2f | Area: %.2f%%\n",
                            vision.getDetectedClass(),
                            vision.getTX(),
                            vision.getTY(),
                            vision.getTargetArea()
                    );
                } else {
                    System.out.println("[LOG] No targets visible.");
                }
            } else {
                System.out.println("[LOG] Waiting for connection...");
            }

            try { Thread.sleep(500); } catch (InterruptedException e) {}
        }
    }
}
