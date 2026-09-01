package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.NinjasLogger;
import frc.robot.Simulation;

public class Indexer extends SubsystemBase {
    private double motorPercent = 0.0;
    private double ballTravelProgress = 0.0;
    private final double LAUNCH_THRESHOLD = 1.0;

    public void setPercent(double percent) {
        motorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        if (motorPercent > 0.1) {
            // Increment how close the ball is to reaching the flywheel wheel contact zone
            ballTravelProgress += motorPercent * 0.1;

            if (ballTravelProgress >= LAUNCH_THRESHOLD) {
                ballTravelProgress = 0.0; // Reset indexer slot

                Simulation.launchBall();
            }
        } else if (motorPercent <= 0.1) {
            ballTravelProgress = 0.0;
        }

        NinjasLogger.log("Indexer/Progress", ballTravelProgress);
    }
}