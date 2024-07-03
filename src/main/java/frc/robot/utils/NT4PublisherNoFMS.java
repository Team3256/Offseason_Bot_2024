package frc.robot.utils;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

// subclass of NT4Publisher that does not publish to NT if FMS is attached. 
// may alleviate some command overrun issues
public class NT4PublisherNoFMS extends NT4Publisher {
    public NT4PublisherNoFMS() {
        super();
    }

    @Override
    public void putTable(LogTable table) {
        if (Robot.isReal() && DriverStation.isFMSAttached()) {
            return;
        }
        super.putTable(table);
    }

}
