// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public abstract class DisableSubsystem extends SubsystemBase {
  public final boolean disabled;

  public DisableSubsystem(boolean enable) {
    super();
    this.disabled = !enable;
  }

  @Override
  public Command run(Runnable action) {
    return disabled ? Commands.none() : super.run(action);
  }

  @Override
  public Command runOnce(Runnable action) {
    return disabled ? Commands.none() : super.runOnce(action);
  }

  @Override
  public Command startEnd(Runnable action, Runnable end) {
    return disabled ? Commands.none() : super.startEnd(action, end);
  }

  @Override
  public Command runEnd(Runnable action, Runnable end) {
    return disabled ? Commands.none() : super.runEnd(action, end);
  }

  @Override
  public Command defer(Supplier<Command> supplier) {
    return disabled ? Commands.none() : super.defer(supplier);
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(disabled ? Commands.none() : defaultCommand);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        this.getClass().getSimpleName() + "/CurrentCommand",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");
  }
}
