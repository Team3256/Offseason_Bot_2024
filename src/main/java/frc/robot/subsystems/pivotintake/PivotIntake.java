// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PivotIntake extends SubsystemBase {

  private final PivotIntakeIO pivotIntakeIO;
  private final PivotIntakeIOInputsAutoLogged pivotIntakeIOAutoLogged =
      new PivotIntakeIOInputsAutoLogged();

  public PivotIntake(PivotIntakeIO pivotIntakeIO) {
    this.pivotIntakeIO = pivotIntakeIO;
  }

  @Override
  public void periodic() {
    pivotIntakeIO.updateInputs(pivotIntakeIOAutoLogged);
    Logger.processInputs(getName(), pivotIntakeIOAutoLogged);
  }

  public Command setPosition(double position) {
    return new StartEndCommand(
        () -> pivotIntakeIO.setPosition(position*PivotIntakeConstants.kPivotMotorGearing), null, this);
  }

  public Command setVoltage(double voltage) {
    return new StartEndCommand(
        () -> pivotIntakeIO.setVoltage(voltage), () -> pivotIntakeIO.setVoltage(0), this);
  }

  public Command off() {
    return new StartEndCommand(() -> pivotIntakeIO.off(), null, this);
  }

  public Command slamZero() {
    return new Command() {
      @Override
      public void initialize() {
        pivotIntakeIO.setVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage);
      }

      @Override
      public void end(boolean interrupted) {
        pivotIntakeIO.off();
        if (!interrupted) {
          pivotIntakeIO.zero();
        }
      }

      @Override
      public boolean isFinished() {
        return pivotIntakeIOAutoLogged.pivotIntakeMotorStatorCurrent
            > PivotIntakeConstants.kPivotSlamStallCurrent;
      }
    };
  }

  public Command slamAndPID() {
    return new SequentialCommandGroup(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return new StartEndCommand(() -> pivotIntakeIO.zero(), null, this);
  }
}
