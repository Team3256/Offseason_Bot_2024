// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends DebugCommandBase {
  private final Shooter shooterSubsystem;

  private final double shooterRps;

  private final double shooterFollowerRps;

  public Shoot(Shooter shooterSubsystem, double shooterRps, double shooterFollowerRps) {
    this.shooterSubsystem = shooterSubsystem;
    this.shooterRps = shooterRps;
    this.shooterFollowerRps = shooterFollowerRps;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    shooterSubsystem.setShooterRps(shooterRps);
    shooterSubsystem.setShooterFollowerRps(shooterFollowerRps);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooterSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
