// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIn extends DebugCommandBase {
  private final Intake intakeSubsystem;
  private TimedBoolean isBeamBreakTriggered;
  private TimedBoolean isBeamBreakTriggeredAuto;
  private TimedBoolean motorVelocitySpiking;
  private double init;

  public IntakeIn(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.motorVelocitySpiking =
        new TimedBoolean(
            intakeSubsystem::isIntakeMotorVelocitySpiking,
            IntakeConstants.kIntakeSequenceTimeThreshold);
    this.isBeamBreakTriggered =
        new TimedBoolean(intakeSubsystem::isBeamBroken, IntakeConstants.kBeamBreakDelayTime);
    this.isBeamBreakTriggeredAuto =
        new TimedBoolean(intakeSubsystem::isBeamBroken, IntakeConstants.kBeamBreakDelayTime / 2);

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.setOutputVoltagePassthrough(-8);
    intakeSubsystem.setOutputVoltageIntake(16);
  }

  @Override
  public void execute() {
    if (IntakeConstants.kUseBeamBreak) {
      isBeamBreakTriggered.update();
      isBeamBreakTriggeredAuto.update();
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
    //   return isBeamBreakTriggeredAuto.hasBeenTrueForThreshold();
    // }
    // else {
    if (IntakeConstants.kUseBeamBreak) {
      if (IntakeConstants.kDelayBeamBreak) {
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
