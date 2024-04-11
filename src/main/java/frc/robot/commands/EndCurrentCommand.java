// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.DebugCommandBase;

public class EndCurrentCommand extends DebugCommandBase {
  Command currentCommand;

  public EndCurrentCommand(Command command) {
    currentCommand = command;
    currentCommand.end(true);
  }
}
