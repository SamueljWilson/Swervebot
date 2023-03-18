package frc.robot;

import edu.wpi.first.math.MathUtil;

public class TrapezoidalConstraint {
    private final double m_maxSpeed;
    private final double m_maxAcceleration;

    public TrapezoidalConstraint(double maxSpeed, double maxAcceleration) {
        m_maxSpeed = maxSpeed;
        m_maxAcceleration = maxAcceleration;
    }

    public double calculate(double desiredVelocity, double currentVelocity, double deltaT) {
        double acceleration = MathUtil.clamp((desiredVelocity - currentVelocity) / deltaT, -m_maxAcceleration, m_maxAcceleration);
        double deltaV = acceleration * deltaT;
        double newVelocity = currentVelocity + deltaV;
        return MathUtil.clamp(newVelocity, -m_maxSpeed, m_maxSpeed);
    }  
}
