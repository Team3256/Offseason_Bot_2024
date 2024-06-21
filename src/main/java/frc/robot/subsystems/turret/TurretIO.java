package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double turretMotorVoltage = 0.0;
        public double turretMotorPosition = 0.0;
        public double turretMotorStatorCurrent = 0.0;
        public double turretMotorSupplyCurrent = 0.0;
        public double turretMotorTemperature = 0.0;
        public double turretMotorReferenceSlope = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setPosition(double position) {}

    public default void zero() {}

    public default void off() {}
}
