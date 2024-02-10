package frc.robot;

import edu.wpi.first.math.MathUtil;

public class TrapezoidalConstraint {
  private final double m_maxSpeed;
  private final double m_maxAcceleration;
  private final double m_maxDeceleration;

  public TrapezoidalConstraint(double maxSpeed, double maxAcceleration, double maxDeceleration) {
    assert(maxSpeed >= 0.0);
    assert(maxAcceleration >= 0.0);
    assert(maxDeceleration >= 0.0); // We negate this during calculation.
    m_maxSpeed = maxSpeed;
    m_maxAcceleration = maxAcceleration;
    m_maxDeceleration = maxDeceleration;
  }

  public double calculate(double desiredVelocity, double currentVelocity, double deltaT) {
    double acceleration = MathUtil.clamp((desiredVelocity - currentVelocity) / deltaT, -m_maxDeceleration, m_maxAcceleration);
    double deltaV = acceleration * deltaT;
    double newVelocity = currentVelocity + deltaV;
    return MathUtil.clamp(newVelocity, -m_maxSpeed, m_maxSpeed);
  }
}