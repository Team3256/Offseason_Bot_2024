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
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

  private final EncoderIO encoderIO;
  private final EncoderIOInputsAutoLogged encoderIOInputs = new EncoderIOInputsAutoLogged();

  public Turret(TurretIO turretIO, EncoderIO encoderIO) {
    this.turretIO = turretIO;
    this.encoderIO = encoderIO;
    this.setDefaultCommand(reset());
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretIOInputs);
    Logger.processInputs(this.getClass().getSimpleName(), turretIOInputs);
    encoderIO.updateInputs(encoderIOInputs);
    Logger.processInputs(this.getClass().getSimpleName()+"/encoder", encoderIOInputs);
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
