// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.Arrays;

public class TurretHelper {
  public static int chineseRemainder(int[] n, double[] a) {
    int prod = Arrays.stream(n).reduce(1, (x, y) -> x * y);
    int sum = 0;
    for (int i = 0; i < n.length; i++) {
      int p = prod / n[i];
      sum += a[i] * mulInv(p, n[i]) * p;
    }
    return sum % prod;
  }

  public static int mulInv(int a, int b) {
    int b0 = b;
    int x0 = 0, x1 = 1;
    if (b == 1) return 1;
    while (a > 1) {
      int q = a / b;
      int tmp = b;
      b = a % b;
      a = tmp;
      tmp = x0;
      x0 = x1 - q * x0;
      x1 = tmp;
    }
    if (x1 < 0) x1 += b0;
    return x1;
  }

  public static int getTurretPosition(
      int gearRatioE1, int gearRatioE2, double encoder1, double encoder2) {
    int[] n = {gearRatioE1, gearRatioE2};
    double[] a = {encoder1, encoder2};
    return chineseRemainder(n, a);
  }
}
