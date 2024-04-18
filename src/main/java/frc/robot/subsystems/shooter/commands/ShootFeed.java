// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShootFeed extends DebugCommandBase {
  private final Shooter shooterSubsystem;

  public ShootFeed(Shooter shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    shooterSubsystem.setShooterRps(ShooterConstants.kShooterFeederRPS);
    shooterSubsystem.setShooterFollowerRps(ShooterConstants.kShooterFollowerFeederRPS);
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
