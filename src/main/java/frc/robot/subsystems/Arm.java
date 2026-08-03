package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Simulation;

public class Arm extends SubsystemBase {
    private double motorPercent = 0.0;

    // Simulation parameters
    private static final double kGearRatio = 48.0;
    private static final double kArmMOI = 0.5; // Moment of Inertia in kg * m^2
    private static final double kArmLengthMeters = 0.6;
    private static final double kMinAngleRad = Math.toRadians(-360.0); // Min angle (e.g., straight down)
    private static final double kMaxAngleRad = Math.toRadians(360.0);  // Max angle
    private static final boolean kSimulateGravity = true;

    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        kGearRatio,
        kArmMOI,
        kArmLengthMeters,
        kMinAngleRad,
        kMaxAngleRad,
        kSimulateGravity,
        0.0
    );

    private PIDController pidArm = new PIDController(1, 0.2,0);
    private Rotation2d goal = Rotation2d.kZero;

    public Arm() {
        pidArm.setIZone(4);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_armSim.getAngleRads());
    }

    public boolean atGoal() {
        return Math.abs(goal.getDegrees() - getAngle().getDegrees()) < 2;
    }

    public void setPercent(double percent) {
        this.motorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    public Command setAngle(Rotation2d angle) {
        return Commands.runOnce(() -> goal = angle);
    }

    @Override
    public void periodic() {
        setPercent(pidArm.calculate(getAngle().getRadians(), goal.getRadians()));

        m_armSim.setInputVoltage(motorPercent * 12.0);
        m_armSim.update(0.02);

        Simulation.updateArmVisual(m_armSim.getAngleRads());

    }
}