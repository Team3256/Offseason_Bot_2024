// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivotintake.PivotIntake;

public class IntakeOutArmOff extends DebugCommandBase {
  private Intake intakeSubsystem;
  private PivotIntake pivotSubsystem;

  public IntakeOutArmOff(Intake intakeSubsystem, PivotIntake pivotSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    pivotSubsystem.off();
    // Wait a bit so off can propagate.
    Timer.delay(0.05);
    // intakeSubsystem.setIntakeVelocity(-2500);
    intakeSubsystem.setOutputVoltageIntake(-16);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    // intakeSubsystem.setDutyCycle(0);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    if (Robot.isReal()) {
      // if (IntakeConstants.kUseTimeOfFlight) {
      // return !(intakeSubsystem.isTOFTriggered());
      // } else {
      // return !(intakeSubsystem.isIntakeMotorCurrentSpiking());
      // }
      // } else {
      // return (intakeSubsystem
      // .isIntakeMotorCurrentSpiking()); // in Sim, you cannot put a load on the motor.
      // This is
      // to allow
      // // you to finish the command in sim
      return false;
    }
    return false;
  }
}
