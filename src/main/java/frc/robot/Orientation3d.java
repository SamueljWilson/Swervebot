package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Orientation3d {
    private final double m_pitch;
    private final double m_roll;
    private final double m_yaw;

    public Orientation3d(double pitch, double roll, double yaw) {
        m_pitch = pitch;
        m_roll = roll;
        m_yaw = yaw;
    }

    public double getPitch() {
        return m_pitch;
    }

    public double getRoll() {
        return m_roll;
    }

    public double getYaw() {
        return m_yaw;
    }

    public double getTilt() {
        // Finds the angle relative to the vertical axis (z)
        double dotProduct = Math.cos(m_pitch) * Math.cos(m_roll);
        double vectorMagnitude = Math.sqrt(
          Math.pow(Math.sin(m_pitch)*Math.cos(m_roll), 2) +
          Math.pow(Math.cos(m_pitch)*Math.sin(m_roll), 2) +
          Math.pow(Math.cos(m_pitch)*Math.cos(m_roll), 2)
        );
        SmartDashboard.putNumber("dotProduct/Magnitude", dotProduct/vectorMagnitude);
        // SmartDashboard.putNumber("vectorMagnitude", vectorMagnitude);
        return Math.acos(dotProduct / vectorMagnitude);
      }
    
    public double getTiltDirection() {
        assert(m_pitch == MathUtil.angleModulus(m_pitch));
        assert(m_roll == MathUtil.angleModulus(m_roll));
        double angle = Math.atan(
            (Math.cos(m_pitch)*Math.sin(m_roll))
            / (Math.sin(m_pitch)*Math.cos(m_roll))
        );
        if (m_pitch >= 0) {
            if (m_roll >= 0) {
                //Q2
                return Math.PI - angle;
            } else {
                //Q3
                return -Math.PI - angle;
            }
        } else {
            //Q1 or Q4
            return -angle; 
        }
    }

    public boolean isTiltedUp() {
        double tiltDir = getTiltDirection();
        // Q2 or Q3
        return (tiltDir <= -Math.PI/2 || tiltDir >= Math.PI/2);
    }
}
