package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;

import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {
    private DoubleSupplier driverLeftX, driverLeftY, driverRightX, driverRightY;
    private boolean enabled;

    public SwerveSubsystem(boolean enabled, DoubleSupplier driverLeftX, DoubleSupplier driverLeftY, DoubleSupplier driverRightX, DoubleSupplier driverRightY) {
        this.enabled = enabled;
        this.driverLeftX = driverLeftX;
        this.driverLeftY = driverLeftY;
        this.driverRightX = driverRightX;
        this.driverRightY = driverRightY;

        if (enabled)
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
    }

    private SwerveSpeeds getDriveInput() {
        Translation2d driverTranslation = new Translation2d(
            -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband),
            -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband)
        );
        driverTranslation = new Translation2d(Math.pow(driverTranslation.getNorm(), GeneralConstants.Swerve.kDriverPowFactor), driverTranslation.equals(new Translation2d()) ? Rotation2d.kZero : driverTranslation.getAngle())
            .times(GeneralConstants.Swerve.kDriverSpeedFactor)
            .times(SubsystemConstants.kSwerve.limits.maxSpeed);

        return new SwerveSpeeds(
            driverTranslation.getX(),
            driverTranslation.getY(),
            -Math.signum(driverRightX.getAsDouble()) * Math.pow(MathUtil.applyDeadband(Math.abs(driverRightX.getAsDouble()), GeneralConstants.Swerve.kJoystickDeadband), GeneralConstants.Swerve.kDriverRotationPowFactor) * GeneralConstants.Swerve.kDriverRotationSpeedFactor * SubsystemConstants.kSwerve.limits.maxAngularVelocity,
            GeneralConstants.Swerve.kDriverFieldRelative
        );
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        Swerve.getInstance().drive(getDriveInput());
        Swerve.getInstance().periodic();
    }
}