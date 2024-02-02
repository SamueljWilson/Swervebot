package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableConstant {
  private String key;
  private double defaultValue;
  private double previousValue;
  
  public TunableConstant(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
  }

  public double get() {
    previousValue = SmartDashboard.getNumber(key, defaultValue);
    return previousValue;
  }

  private double peek() {
    return SmartDashboard.getNumber(key, defaultValue);
  }

  public boolean hasChanged() {
    double currentValue = peek();
    return currentValue != previousValue;
  }
}
