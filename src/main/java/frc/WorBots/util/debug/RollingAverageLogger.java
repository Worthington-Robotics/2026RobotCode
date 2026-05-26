// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import edu.wpi.first.wpilibj.DriverStation;

public class RollingAverageLogger {
  private double value = 0.0;
  private int entries = 0;
  private String table;
  private String key;

  public RollingAverageLogger(String table, String key) {
    this.table = table;
    this.key = key;
  }

  public void addValue(double value) {
    if(DriverStation.isDisabled()){
      return;
    }
    this.value += value;
    entries++;
    NTLogger.putNumber(table, key, this.value / entries);
  }

  public double get() {
    return value / entries;
  }

  public void reset() {
    value = 0;
    entries = 0;
  }
}
