// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.commands;

import static frc.robot.subsystems.intake.IntakeConstants.kIntakeIntakeVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kPassthroughIntakeVoltage;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeInTeleop extends DebugCommandBase {
  private final Intake intakeSubsystem;
  private TimedBoolean isBeamBreakTriggered;
  private TimedBoolean motorVelocitySpiking;
  private double init;

  public IntakeInTeleop(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.motorVelocitySpiking =
        new TimedBoolean(
            intakeSubsystem::isIntakeMotorVelocitySpiking,
            IntakeConstants.kIntakeSequenceTimeThreshold);
    this.isBeamBreakTriggered = new TimedBoolean(intakeSubsystem::isBeamBroken, 0.04);

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.setOutputVoltagePassthrough(kPassthroughIntakeVoltage);
    intakeSubsystem.setOutputVoltageIntake(kIntakeIntakeVoltage);
  }

  @Override
  public void execute() {
    if (IntakeConstants.kUseBeamBreak) {
      isBeamBreakTriggered.update();
    } else {
      motorVelocitySpiking.update();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    // if (DriverStation.isAutonomousEnabled()) {
    // return isBeamBreakTriggeredAuto.hasBeenTrueForThreshold();
    // }
    // else {
    if (IntakeConstants.kUseBeamBreak) {
      if (true) {
        return isBeamBreakTriggered.hasBeenTrueForThreshold();
      } else {
        return intakeSubsystem.isBeamBroken();
      }
    } else if (IntakeConstants.kUseTimedCurrentSpike) {
      return (System.currentTimeMillis() - init) > 1000
          && motorVelocitySpiking.hasBeenTrueForThreshold();
    } else {
      return intakeSubsystem.isIntakeMotorCurrentSpiking();
    }
    // }
  }
}
