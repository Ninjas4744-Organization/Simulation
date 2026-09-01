package frc.robot.constants;

public class GeneralConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static class Swerve {
        public static double kDriverSpeedFactor = 1;
        public static double kDriverRotationSpeedFactor = 1;
        public static double kDriverPowFactor = 0.75;
        public static double kDriverRotationPowFactor = 0.75;
        public static double kJoystickDeadband = 0.04;
        public static boolean kDriverFieldRelative = true;
    }
}
