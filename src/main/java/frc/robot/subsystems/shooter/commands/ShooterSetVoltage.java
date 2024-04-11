// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSetVoltage extends DebugCommandBase {
  private final Shooter shooterSubsystem;
  private final double volts;
  private final boolean forever;

  public ShooterSetVoltage(Shooter shooterSubsystem, double volts, boolean forever) {
    this.shooterSubsystem = shooterSubsystem;
    this.volts = volts;
    this.forever = forever;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    shooterSubsystem.setInputShooterVoltage(volts);
  }

  @Override
  public void end(boolean interrupted) {
    if (forever) shooterSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return !forever;
  }
}
