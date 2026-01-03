package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionTestHarness {
    public static void main(String[] args) {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        // Stop client mode and start as the Server
        nt.stopClient();
        nt.startServer();

        System.out.println("Laptop is now the NT Server. Waiting for Limelight...");

        // 2025-style Connection Listener
        nt.addConnectionListener(true, event -> {
            if (event.is(NetworkTableEvent.Kind.kConnected)) {
                System.out.println("[CONNECTED] Limelight has joined the table!");
            }
        });

        while (true) {
            var table = nt.getTable("limelight");

            // tv: 1 if a target is visible, 0 if not
            boolean hasTarget = table.getEntry("tv").getDouble(0.0) == 1.0;

            if (hasTarget) {
                // Horizontal offset (tx) and Vertical offset (ty)
                double tx = table.getEntry("tx").getDouble(0.0);
                double ty = table.getEntry("ty").getDouble(0.0);

                // tclass: The string label of the detected object (e.g., "cube", "cone")
                String detectedClass = table.getEntry("tclass").getString("None");
                String rawJson = table.getEntry("json").getString("");
                if (!rawJson.isEmpty()) {
                    System.out.println("RAW DATA: " + rawJson);
                }

                // ta: The target area (0% to 100% of the image)
                double area = table.getEntry("ta").getDouble(0.0);

                System.out.printf("[DETECTED] Class: %s | TX: %.2f | TY: %.2f | Area: %.2f%%\n",
                        detectedClass, tx, ty, area);
            } else {
                System.out.println("Searching for objects...");
            }

            try { Thread.sleep(200); } catch (InterruptedException e) {} // 5Hz log rate
        }
    }
}