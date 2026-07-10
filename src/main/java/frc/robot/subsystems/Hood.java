package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Simulation;

public class Hood extends SubsystemBase {
    private double motorPercent = 0.0;
    private double angleRad = 0.0;
    private double velocityRadPerSec = 0.0;

    private final double DT = 0.02;
    private final double MAX_ACCEL = 20.0;
    private final double GRAVITY_COS_APPROX = 2.0;
    private final double MIN_ANGLE = Units.degreesToRadians(15);
    private final double MAX_ANGLE = Units.degreesToRadians(75);

    public void setPercent(double percent) {
        motorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleRad);
    }

    @Override
    public void periodic() {
        double acceleration = (motorPercent * MAX_ACCEL) - (GRAVITY_COS_APPROX * Math.cos(angleRad));
        velocityRadPerSec = (velocityRadPerSec + acceleration * DT) * 0.99;
        angleRad = angleRad + (velocityRadPerSec * DT);
        angleRad = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleRad));

        if (angleRad == MIN_ANGLE || angleRad == MAX_ANGLE)
            velocityRadPerSec = 0.0;

        // Sync with the visual Canvas
        Simulation.updateHoodVisual(angleRad);
    }
}