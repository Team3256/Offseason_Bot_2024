// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbIOAutoLogged = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), climbIOAutoLogged);
  }

  public Command setPosition(double position) {
    return new StartEndCommand(
        () -> climbIO.setPosition(position * ClimbConstants.gearRatio), () -> {}, this);
  }

  public Command setVoltage(double voltage) {
    return new StartEndCommand(
        () -> climbIO.setVoltage(voltage), () -> climbIO.setVoltage(0), this);
  }

  public Command off() {
    return new StartEndCommand(() -> climbIO.off(), () -> {}, this);
  }

  public Command zero() {
    return new StartEndCommand(() -> climbIO.zero(), () -> {}, this);
  }

  public Command extendClimber() {
    return setPosition(ClimbConstants.kClimbUpPosition);
  }

  public Command retractClimber() {
    return setPosition(ClimbConstants.kClimbDownPosition);
  }
}
