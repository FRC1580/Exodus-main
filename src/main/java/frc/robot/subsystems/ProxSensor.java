package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class ProxSensor {
    private final AnalogInput proximitySensor = new AnalogInput(0);
    private static final double V_SUPPLY = 12.0; // Change if using a different voltage

    public double getDistance() {
        double voltage = proximitySensor.getVoltage();
        double distance = (voltage * 1000) / V_SUPPLY; // Distance in mm
        return distance / 10; // Convert mm to cm
    }

    public void periodic() {
        System.out.println("Proximity Sensor Distance: " + getDistance() + " cm");
    }
}
