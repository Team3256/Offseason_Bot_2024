// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.vision.Vision;

public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;
    this.setDefaultCommand(reset());
  }

  public Command setPositionRelativeToSwerve(Rotation2d position, Rotation2d swerveAngle) {
    return new StartEndCommand(
        () -> turretIO.setPosition(position.getDegrees() - swerveAngle.getDegrees()),
        () -> {},
        this);
  }

  public Command setPosition(Rotation2d position) {
    return new StartEndCommand(() -> turretIO.setPosition(position.getDegrees()), () -> {}, this);
  }

  public Command zero() {
    return new StartEndCommand(() -> turretIO.zero(), () -> {}, this);
  }

  public Command followLimelight(Vision vision) {
    return new PIDCommand(
        new PIDController(1, 0, 0),
        vision.getCompensatedCenterLimelightXSupplier(),
        0,
        (output) -> turretIO.setVoltage(output),
        this);
  }

  public Command reset() {
    return new StartEndCommand(
        () -> {
          if (turretIOInputs.turretMotorPosition > TurretConstants.kForwardLimit) {
            turretIO.setPosition(0);
          } else if (turretIOInputs.turretMotorPosition < TurretConstants.kReverseLimit) {
            turretIO.setPosition(0);
          }
        },
        () -> {},
        this);
  }
}
