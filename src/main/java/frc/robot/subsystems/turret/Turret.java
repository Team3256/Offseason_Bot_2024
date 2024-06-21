package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO turretIO) {
        this.turretIO = turretIO;
    }
}
