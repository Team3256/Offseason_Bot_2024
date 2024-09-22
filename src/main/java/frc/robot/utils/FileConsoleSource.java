// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.BufferOverflowException;
import java.nio.CharBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import org.littletonrobotics.junction.console.ConsoleSource;

/**
 * Reads file data on the RIO. Saves stdout to WPILog console. Useful for checking if CANivore is
 * disconnected. (CANivore API is slow and blocking. Checking it in a periodic() loop will lag the
 * robot.)
 */
public class FileConsoleSource implements ConsoleSource {
  private final String filePath;
  private final Thread thread;
  private final String logName;
  private final BlockingQueue<String> queue = new ArrayBlockingQueue<>(100);
  private final List<String> lines = new ArrayList<>();

  public FileConsoleSource(String filePath) {
    this.filePath = filePath;
    this.logName = Integer.toHexString(this.hashCode());
    thread = new Thread(this::run, "FileConsoleSource." + Integer.toHexString(this.hashCode()));
    thread.setDaemon(true);
    thread.start();
  }

  public FileConsoleSource(String filePath, String logName) {
    this.filePath = filePath;
    this.logName = logName;
    thread = new Thread(this::run, "FileConsoleSource." + logName);
    thread.setDaemon(true);
    thread.start();
  }

  public String getLogName() {
    return logName;
  }

  public String getNewData() {
    lines.clear();
    queue.drainTo(lines);
    return String.join("\n", lines);
  }

  public void close() throws Exception {
    thread.interrupt();
  }

  private void run() {
    // Initialize reader
    CharBuffer buffer = CharBuffer.allocate(10240);
    BufferedReader reader;
    try {
      reader = new BufferedReader(new FileReader(filePath));
    } catch (FileNotFoundException e) {
      DriverStation.reportError(
          "Failed to open console file \"" + filePath + "\", disabling console capture.", true);
      return;
    }

    while (true) {
      // Read new data from console
      while (true) {
        int nextChar = -1;
        try {
          nextChar = reader.read();
        } catch (IOException e) {
          DriverStation.reportError(
              "Failed to read console file \"" + filePath + "\", disabling console capture.", true);
          try {
            reader.close();
          } catch (IOException io) {
          }
          return;
        }
        if (nextChar != -1) {
          try {
            buffer.put((char) nextChar);
          } catch (BufferOverflowException e) {
          }
        } else {
          // Break read loop, send complete lines to queue
          break;
        }
      }

      // Read all complete lines
      String output = null;
      for (int i = buffer.position(); i > 0; i--) {
        if (i < buffer.position() && buffer.get(i) == '\n') {
          int originalPosition = buffer.position();
          output = new String(buffer.array(), 0, i);
          buffer.rewind();
          buffer.put(buffer.array(), i + 1, buffer.limit() - i - 1);
          buffer.position(originalPosition - i - 1);
          break;
        }
      }
      if (output != null) {
        try {
          queue.put(output);
        } catch (InterruptedException e) {
          try {
            reader.close();
          } catch (IOException io) {
          }
          return;
        }
      }

      // Sleep to avoid spinning needlessly
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        try {
          reader.close();
        } catch (IOException io) {
        }
        return;
      }
    }
  }
}
