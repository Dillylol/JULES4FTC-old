package org.firstinspires.ftc.teamcode.common;

/**
 * Constants for the Swyft Ranger distance sensor.
 * Includes calibration formulas for different FOV modes.
 */
public final class SwyftRangerConstants {
    private SwyftRangerConstants() {}

    public static final String HARDWARE_NAME = "ranger";

    // Voltage range from documentation: 0.1V - 3.2V
    public static final double MIN_VOLTAGE = 0.1;
    public static final double MAX_VOLTAGE = 3.2;

    /**
     * Calculates inches using the unified regression pattern (20 FOV).
     * Formula: (Voltage * 48.78136376) - 4.985354503
     */
    public static double voltageToInches(double sensorVoltage, double batteryVoltage) {
        // Battery compensation removed in favor of single regression pattern provided by user.
        // Keeping signature for compatibility, but ignoring batteryVoltage for now.
        return (sensorVoltage * 48.78136376) - 4.985354503;
    }

    /**
     * Default converter.
     */
    public static double voltageToInches(double voltage) {
        return (voltage * 48.78136376) - 4.985354503;
    }

    public static double voltageToCm(double voltage) {
        return voltageToInches(voltage) * 2.54;
    }

    public static double voltageToCm(double sensorVoltage, double batteryVoltage) {
        return voltageToInches(sensorVoltage, batteryVoltage) * 2.54;
    }
}
