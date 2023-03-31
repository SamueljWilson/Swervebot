// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

/** Add your docs here. */
public class ArmInterp {
  static class HtEntry{double cycles; double height; public HtEntry(double c, double h) {cycles = c; height = h;}}
  static final HtEntry[] htTable = {
      new HtEntry(0, 0), //TODO: REPLACE TABLE
      new HtEntry(23.1, 0.21),
      new HtEntry(25.1, 0.215),
      new HtEntry(26.8, 0.225),
      new HtEntry(27.4, 0.23),
      new HtEntry(28.5, 0.24),
      new HtEntry(29.9, 0.245),
      new HtEntry(30.9, 0.255),
      new HtEntry(32.1, 0.26),
      new HtEntry(33.1, 0.27),
      new HtEntry(34.3, 0.28),
      new HtEntry(37.2, 0.32),
      new HtEntry(38.3, 0.33),
      new HtEntry(39.6, 0.345),
      new HtEntry(40.7, 0.36),
      new HtEntry(41.9, 0.375),
      new HtEntry(42.9, 0.38),
      new HtEntry(43.2, 0.39),
      new HtEntry(45.3, 0.415),
      new HtEntry(46.3, 0.425),
      new HtEntry(47.3, 0.44),
      new HtEntry(48.2, 0.455),
      new HtEntry(49.8, 0.48),
      new HtEntry(50.8, 0.5),
      new HtEntry(51.4, 0.51),
      new HtEntry(52.3, 0.53),
      new HtEntry(54.5, 0.575),
      new HtEntry(55.8, 0.6),
      new HtEntry(56.9, 0.625),
      new HtEntry(57.8, 0.645),
      new HtEntry(59.4, 0.68),
      new HtEntry(61.5, 0.72),
      new HtEntry(62.3, 0.74),
      new HtEntry(63.8, 0.77),
      new HtEntry(64.7, 0.795),
      new HtEntry(65.6, 0.82),
      new HtEntry(67.0, 0.86),
      new HtEntry(68.5, 0.9),
      new HtEntry(69.8, 0.95),
      new HtEntry(71.5, 0.99),
      new HtEntry(72.6, 1.03),
      new HtEntry(73.5, 1.05),
      new HtEntry(75.2, 1.1),
      new HtEntry(76.9, 1.15),
      new HtEntry(77.3, 1.16),
      new HtEntry(78.0, 1.18),
      new HtEntry(79.6, 1.23),
      new HtEntry(80.1, 1.245),
      new HtEntry(81.7, 1.295),
      new HtEntry(82.3, 1.325),
      new HtEntry(83.7, 1.37),
      new HtEntry(84.4, 1.39),
      new HtEntry(85.2, 1.42),
      new HtEntry(86.3, 1.46),
      new HtEntry(87.1, 1.475),
      new HtEntry(88.3, 1.51),
      new HtEntry(89.5, 1.56),
      new HtEntry(91.2, 1.6)
  };

  public static double getMaxCycles() {
    return htTable[htTable.length-1].cycles;
  }

  public static int cycleIndex(double cycles) {
    assert(cycles >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(cycles, 0), (a,b) -> Double.compare(a.cycles, b.cycles));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    return index;
  }

  public static double cyclesToHeight(double cycles) {
    int index = cycleIndex(cycles);
    if (index == 0) {
      return htTable[index].height;
    } else if (index == htTable.length) {
      return htTable[htTable.length-1].height;
    } else if (htTable[index].cycles == cycles) {
      return htTable[index].height;
    }
    // Linear Interpolation
    else {
    double c0 = htTable[index-1].cycles;
    double c1 = htTable[index].cycles;
    double h0 = htTable[index-1].height;
    double h1 = htTable[index].height;
    double scaler = (cycles-c0) / (c1 - c0);
    double heightRange = h1 - h0;
    return h0 + scaler*heightRange;
    }
  }

  public static int heightIndex(double height) {
    assert(height >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(0, height), (a,b) -> Double.compare(a.height, b.height));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    assert(index < htTable.length);
    return index;
  }
  
  public static double heightToCycles(double height) {
    int index = heightIndex(height);
    if (index == 0) {
      return htTable[index].cycles;
     }
     else if (htTable[index].height == height) {
       return htTable[index].cycles;
     }
     // Linear Interpolation
     else {
      double c0 = htTable[index-1].cycles;
      double c1 = htTable[index].cycles;
      double h0 = htTable[index-1].height;
      double h1 = htTable[index].height;
      double scaler = (height-h0) / (h1 - h0);
      double cycleRange = c1 - c0;
      double cycles = c0 + scaler*cycleRange;
      return cycles;
     }
  }

  public static double getCyclesPerMeter(double height) {
    if (height == 0.0) {
      return 0.0;
    }
    int indexGE = heightIndex(height);
    assert(indexGE > 0);
    int indexLT = indexGE - 1;
    double deltaC = htTable[indexGE].cycles - htTable[indexLT].cycles;
    double deltaH = htTable[indexGE].height - htTable[indexLT].height;
    return deltaC / deltaH;
  }

  public static double vheightToRPM(double metersPerSecond, double height) {
    double slope = getCyclesPerMeter(height);
    double secondsPerMinute = 60;
    return slope * metersPerSecond * secondsPerMinute;
  }
}
