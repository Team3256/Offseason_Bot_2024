package frc.robot.subsystems.ampbar;

import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {
    @AutoLog
    public static class AmpBarIOInputs {
        public double ampBarMotorVoltage = 0.0;
        public double ampBarMotorPosition = 0.0;
        public double ampBarMotorVelocity = 0.0;
        public double ampBarMotorStatorCurrent = 0.0;
        public double ampBarMotorSupplyCurrent = 0.0;
        public double ampBarMotorTemperature = 0.0;
    }

    public default void updateInputs(AmpBarIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {}

    public default boolean isCurrentSpiking() {
        return false;
    }

    public default void off() {}
}
