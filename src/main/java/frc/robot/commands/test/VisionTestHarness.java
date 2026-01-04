package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.VisionSubsystem;

public class VisionTestHarness {
    public static void main(String[] args) {
        // Must initialize HAL to use WPILib components like NetworkTables
        edu.wpi.first.hal.HAL.initialize(500, 0);

        // Start as the Server so the Limelight (acting as Client) can connect
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.stopClient();
        nt.startServer();

        // Instantiate the refactored subsystem
        VisionSubsystem vision = new VisionSubsystem();

        System.out.println("--- NT Server Active ---");
        System.out.println("Check Limelight Dashboard: Set 'Custom NT Server IP' to your Laptop IP");
        System.out.println("Waiting for Limelight connection...");

        while (true) {
            // Check if any client (the Limelight) has connected to our laptop
            if (nt.getConnections().length > 0) {
                if (vision.hasTarget()) {
                    // Logs data using the working 2025 targets_Detector APIs
                    System.out.printf("[LIVE] Object: %-10s | TX: %6.2f | TY: %6.2f | Area: %5.2f%%\n",
                            vision.getDetectedClass(),
                            vision.getTX(),
                            vision.getTY(),
                            vision.getTargetArea()
                    );
                } else {
                    System.out.println("[LIVE] Searching... (No targets in view)");
                }
            } else {
                // Helps debug network location/static IP issues
                System.out.println("[OFFLINE] Waiting for Limelight to join NetworkTable...");
            }

            try {
                Thread.sleep(200); // Increased to 5Hz for smoother terminal output
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }
}