// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.TimedBoolean;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeIOAutoLogged = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs(getName(), intakeIOAutoLogged);
  }

  public Command setVoltage(double voltage, double passthroughVoltage) {
    return new StartEndCommand(
        () -> {
          intakeIO.setIntakeVoltage(voltage);
          intakeIO.setPassthroughVoltage(passthroughVoltage);
        },
        () -> intakeIO.off(),
        this);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return new StartEndCommand(
        () -> {
          intakeIO.setIntakeVelocity(velocity);
          intakeIO.setPassthroughVelocity(passthroughVelocity);
        },
        () -> intakeIO.off(),
        this);
  }

  public Command setIntakeVoltage(double voltage) {
    return new StartEndCommand(
        () -> intakeIO.setIntakeVoltage(voltage),
        () -> intakeIO.off(),
        this);
  }

  public Command setIntakeVelocity(double velocity) {
    return new StartEndCommand(
        () -> intakeIO.setIntakeVelocity(velocity),
        () -> intakeIO.off(),
        this);
  }

  public Command setPassthroughVoltage(double voltage) {
    return new StartEndCommand(
        () -> intakeIO.setPassthroughVoltage(voltage),
        () -> intakeIO.off(),
        this);
  }

  public Command setPassthroughVelocity(double velocity) {
    return new StartEndCommand(
        () -> intakeIO.setPassthroughVelocity(velocity),
        () -> intakeIO.off(),
        this);
  }

  public Command off() {
    return new StartEndCommand(() -> intakeIO.off(), () -> {}, this);
  }

  public Command intakeIn() {
    return new Command() {
      TimedBoolean beamBreak = new TimedBoolean(intakeIO::isBeamBroken, IntakeConstants.kBeamBreakDelayTime);
      @Override
      public void initialize() {
        intakeIO.setIntakeVoltage(IntakeConstants.kIntakeIntakeVoltage);
        intakeIO.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage);
      }
      @Override
      public void execute() {
        beamBreak.update();
      }

      @Override
      public boolean isFinished() {
        return beamBreak.hasBeenTrueForThreshold();
      }
    };
  }

  public boolean isBeamBroken() {
    return intakeIO.isBeamBroken();
  }

}
