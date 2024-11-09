// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led.commands;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.led.IndicatorAnimation;
import frc.robot.subsystems.led.LED;
import com.fasterxml.jackson.databind.ObjectMapper;

// Please run this at 10 HZ or the animation will not work
public class BadApple extends DebugCommandBase {
    private LED led;
    private int currentFrame;
    private String[][] frames;

    public BadApple(LED led) {
        this.led = led;
        this.currentFrame = 0;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        super.initialize();
        ObjectMapper objectMapper = new ObjectMapper();
        try {
            frames = objectMapper.readValue(new File(Filesystem.getDeployDirectory(), "badapple.json"),
                    String[][].class);
        } catch (IOException e) {
            e.printStackTrace();
            frames = new String[0][0];
        }
        // new CoordinatesButItsMultiple(led, getCoordinates(ledStates), r, g, b,
        // w).schedule();
    }

    @Override
    public void execute() {
        super.execute();
        new CoordinatesButItsMultiple(led, getCoordinates(frames[currentFrame]), 255, 255, 255,
                1).schedule();
        currentFrame++;
        currentFrame %= frames.length;
    }

    private int[][] getCoordinates(String[] ledStates) {

        int[][] coordinates = new int[ledStates.length][2];
        int height = ledStates.length - 1;
        for (int y = 0; y < ledStates.length; y++) {
            String row = ledStates[y];
            for (int x = 0; x < row.length(); x++) {
                if (row.charAt(x) == '1') {
                    coordinates[y] = new int[] { x, height - y };
                }
            }
        }
        return coordinates;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        led.animate(IndicatorAnimation.Default);
    }
}
