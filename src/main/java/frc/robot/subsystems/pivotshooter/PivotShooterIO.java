package frc.robot.subsystems.pivotshooter;

import org.littletonrobotics.junction.AutoLog;

public interface PivotShooterIO {
    @AutoLog
    public static class PivotShooterIOInputs {
        public double pivotShooterMotorVoltage = 0.0;
        public double pivotShooterMotorPosition = 0.0;
        public double pivotShooterMotorDegrees = 0.0;
        public double pivotShooterMotorStatorCurrent = 0.0;
        public double pivotShooterMotorSupplyCurrent = 0.0;
        public double pivotShooterMotorTemperature = 0.0;

        public double lastCenterLimelightTY = 0.0;
        public double lastLastCenterLimelightTY = 0.0;
        public double centerLimelightTYOffset = 0.0;

        public double currentCenterLimelightTY = 0.0;
        public double interpolatedPivotPosition = 0.0;
    }

    public default void updateInputs(PivotShooterIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setVoltage(double voltage) {}

    public default void off() {}

    public default void zero() {}

}