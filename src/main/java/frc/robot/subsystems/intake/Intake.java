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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.helpers.TimedBoolean;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeIOAutoLogged = new IntakeIOInputsAutoLogged();

  private final Trigger debouncedBeamBreak = new Trigger(this::isBeamBroken).debounce(0.1);;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), intakeIOAutoLogged);
  }

  public Command setVoltage(double voltage, double passthroughVoltage) {
    return this.run(()->{
      intakeIO.setIntakeVoltage(voltage);
        intakeIO.setPassthroughVoltage(passthroughVoltage);
    }).finallyDo(intakeIO::off);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return this.run(
        () -> {
          intakeIO.setIntakeVelocity(velocity);
          intakeIO.setPassthroughVelocity(passthroughVelocity);
        }).finallyDo(intakeIO::off);
  }

  public Command setIntakeVoltage(double voltage) {
    return this.run(() -> intakeIO.setIntakeVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setIntakeVelocity(double velocity) {
    return this.run(() -> intakeIO.setIntakeVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command setPassthroughVoltage(double voltage) {
    return this.run(() -> intakeIO.setPassthroughVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setPassthroughVelocity(double velocity) {
    return this.run(() -> intakeIO.setPassthroughVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command off() {
    return this.runOnce(intakeIO::off);
  }

  public Command intakeIn() {
    return this.run(()->{
      intakeIO.setIntakeVoltage(IntakeConstants.kIntakeIntakeVoltage);
      intakeIO.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage);
    }).until(debouncedBeamBreak).andThen(this.off());
  }

  public boolean isBeamBroken() {
    return intakeIO.isBeamBroken();
  }
}
