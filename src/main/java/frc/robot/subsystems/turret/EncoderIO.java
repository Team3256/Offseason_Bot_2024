package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {

    @AutoLog
    public static class EncoderIOInputs {
        public double encoderPosition = 0.0;
        public double encoderVelocity = 0.0;

        public Rotation2d turretMotorAbsolutePosition = new Rotation2d();
    }

    public default void updateInputs(EncoderIOInputs inputs) {}

    public default void zero() {}
}
