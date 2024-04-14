package frc.robot.subsystems.led.commands;

import frc.robot.subsystems.led.IndicatorAnimation;
import frc.robot.subsystems.led.LED;
import frc.robot.helpers.DebugCommandBase;

import static frc.robot.subsystems.led.LEDConstants.kLEDHeight;
import static frc.robot.subsystems.led.LEDConstants.kLEDWidth;

public class SetLEDsFromBinaryString extends DebugCommandBase {
    private LED led;
    private String[] ledStates;

    public SetLEDsFromBinaryString(LED led, String[] ledStates) {
        this.led = led;
        this.ledStates = ledStates;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        super.initialize();
        int height = ledStates.length;
        int width = ledStates[0].length();

        for (int y = 0; y < height; y++) {
            // String row = ledStates[y].replace('B', '0'); // Assume binary character
            // starts with B, replace with 0
            String row = ledStates[y];
            for (int x = 0; x < width; x++) {
                int ledColor = (row.charAt(x) == '1') ? 255 : 0; // White if '1', off if '0'
                led.setLedColor(ledColor, ledColor, ledColor, 0, getIndex(kLEDWidth - x, y) + 8,
                        1);
            }
        }
    }

    private int getIndex(int x, int y) {
        return (kLEDWidth * (y % kLEDHeight)) + (x % kLEDWidth);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        led.animate(IndicatorAnimation.Default);
    }
}