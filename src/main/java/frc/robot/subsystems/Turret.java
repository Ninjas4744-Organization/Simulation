package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Simulation;

public class Turret extends SubsystemBase {
    private double motorPercent = 0.0;
    private double angleRad = 0.0;
    private double velocityRadPerSec = 0.0;

    private final double DT = 0.02;
    private final double MAX_ACCEL = 3.0;

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleRad);
    }

    public void setPercent(double percent) {
        motorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        double acceleration = motorPercent * MAX_ACCEL;
        velocityRadPerSec = (velocityRadPerSec + acceleration * DT) * 0.98;
        angleRad = angleRad + (velocityRadPerSec * DT);
    }
}