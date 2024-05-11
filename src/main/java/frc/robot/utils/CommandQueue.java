package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;

public class CommandQueue extends SubsystemBase {

    private HashMap<BooleanSupplier, Command> commandMap = new HashMap<>();


    public CommandQueue() {
    }



    public void periodic() {
        for (BooleanSupplier trigger : commandMap.keySet()) {
            if (trigger.getAsBoolean()) {
                commandMap.get(trigger).schedule();
                commandMap.remove(trigger);
            }
        }
    }

    public Command addCommand(BooleanSupplier trigger, Command command) {
        return new InstantCommand(()-> commandMap.put(trigger, command));
    }

    public Command addCommand(Trigger trigger, Command command) {
        return addCommand(trigger::getAsBoolean, command);
    }

    public Command removeCommand(BooleanSupplier trigger) {
        return new InstantCommand(()-> commandMap.remove(trigger));
    }






}
