// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PivotShooter extends SubsystemBase {

  private final PivotShooterIO pivotShooterIO;
  private final PivotShooterIOInputsAutoLogged pivotShooterIOAutoLogged =
      new PivotShooterIOInputsAutoLogged();

  public PivotShooter(PivotShooterIO pivotShooterIO) {
    this.pivotShooterIO = pivotShooterIO;
  }

  @Override
  public void periodic() {
    pivotShooterIO.updateInputs(pivotShooterIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), pivotShooterIOAutoLogged);
  }

  public Command setPosition(double position) {
    return new StartEndCommand(
        () -> pivotShooterIO.setPosition(position * PivotShooterConstants.kPivotMotorGearing),
        () -> {},
        this);
  }

  public Command setVoltage(double voltage) {
    return new StartEndCommand(
        () -> pivotShooterIO.setVoltage(voltage), () -> pivotShooterIO.setVoltage(0), this);
  }

  public Command off() {
    return new StartEndCommand(() -> pivotShooterIO.off(), () -> {}, this);
  }

  public Command slamZero() {
    return new Command() {
      @Override
      public void initialize() {
        pivotShooterIO.setVoltage(PivotShooterConstants.kPivotSlamShooterVoltage);
      }

      @Override
      public void end(boolean interrupted) {
        pivotShooterIO.off();
        if (!interrupted) {
          pivotShooterIO.zero();
        }
      }

      @Override
      public boolean isFinished() {
        return pivotShooterIOAutoLogged.pivotShooterMotorStatorCurrent
            > PivotShooterConstants.kPivotSlamStallCurrent;
      }
    };
  }

  public Command slamAndPID() {
    return new SequentialCommandGroup(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return new StartEndCommand(() -> pivotShooterIO.zero(), () -> {}, this);
  }

  public Command bruh() {
    return new RunCommand(
        () ->
            pivotShooterIO.setPosition(
                pivotShooterIOAutoLogged.interpolatedPivotPosition
                    * PivotShooterConstants.kPivotMotorGearing),
        this);
  }
}
