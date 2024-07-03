// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterIOAutoLogged = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), shooterIOAutoLogged);
  }

  public Command setVoltage(double voltage, double followerVoltage) {
    return this.run(
            () -> {
              shooterIO.setShooterVoltage(voltage);
              shooterIO.setShooterFollowerVoltage(followerVoltage);
            })
        .finallyDo(shooterIO::off);
  }

  public Command setVelocity(double velocity, double followerVelocity) {
    return this.run(
            () -> {
              shooterIO.setShooterVelocity(velocity);
              shooterIO.setShooterFollowerVelocity(followerVelocity);
            })
        .finallyDo(shooterIO::off);
  }

  public Command off() {
    return this.runOnce(shooterIO::off);
  }
}
