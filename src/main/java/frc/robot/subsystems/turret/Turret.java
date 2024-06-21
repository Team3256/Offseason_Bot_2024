package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.vision.Vision;

public class Turret extends SubsystemBase {

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO turretIO) {
        this.turretIO = turretIO;
    }

    public Command setPositionRelativeToSwerve(Rotation2d position, Rotation2d swerveAngle) {
        return new StartEndCommand(()->turretIO.setPosition(position.getDegrees() - swerveAngle.getDegrees()), () -> {}, this);
    }

    public Command setPosition(Rotation2d position) {
        return new StartEndCommand(()->turretIO.setPosition(position.getDegrees()), () -> {}, this);
    }

    public Command zero() {
        return new StartEndCommand(()->turretIO.zero(), () -> {}, this);
    }

    public Command followLimelight(Vision vision) {
        return new PIDCommand(
                new PIDController(1,0,0),
                ()->turretIOInputs.turretMotorPosition,
                vision.getCompensatedCenterLimelightXSupplier(),
                (output)->turretIO.setVoltage(output),
                this
        );
    }
}
