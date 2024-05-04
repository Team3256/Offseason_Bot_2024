package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveMotorVoltage = 0.0;
        public double driveMotorVelocity = 0.0;
        public double driveMotorPosition = 0.0;
        public double driveMotorStatorCurrent = 0.0;
        public double driveMotorSupplyCurrent = 0.0;
        public double driveMotorTemperature = 0.0;
        public double driveMotorReferenceSlope = 0.0;


        public Rotation2d absolutePosition = new Rotation2d();

        public double angleMotorVoltage = 0.0;
        public double angleMotorPosition = 0.0;
        public double angleMotorStatorCurrent = 0.0;
        public double angleMotorSupplyCurrent = 0.0;
        public double angleMotorTemperature = 0.0;
        public double angleMotorReferenceSlope = 0.0;

        public SwerveModuleState currentState = new SwerveModuleState();
        public SwerveModulePosition currentPosition = new SwerveModulePosition();

    }

    public default void updateInputs(ModuleIOInputs inputs) {
    }

    public default void setDriveVelocity(double velocity, boolean isOpenLoop) {
    }

    public default void setAnglePosition(double position) {
    }

    public default void resetToAbsolute() {}

    public default SwerveModuleState getState() {return new SwerveModuleState();}

    public default SwerveModulePosition getPosition() {return new SwerveModulePosition();}

    public default Rotation2d getCANcoder() { return new Rotation2d();}

}
