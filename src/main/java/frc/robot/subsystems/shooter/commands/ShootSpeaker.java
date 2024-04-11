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

public class ShootSpeaker extends DebugCommandBase {
  private final Shooter shooterSubsystem;

  public ShootSpeaker(Shooter shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    shooterSubsystem.setShooterRps(ShooterConstants.kShooterSpeakerRPS);
    shooterSubsystem.setShooterFollowerRps(ShooterConstants.kShooterFollowerSpeakerRPS);
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
