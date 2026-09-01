package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.NinjasLogger;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.ExampleSubsystem.ExampleState.*;

public class ExampleSubsystem extends StateMachineBase<ExampleSubsystem.ExampleState> {
    public enum ExampleState {
        UNKNOWN,
        RESET,
        IDLE,
        SHOOT
    }

    private Controller controller;
    private boolean enabled;

    public ExampleSubsystem(boolean enabled) {
        super(ExampleState.class);
        currentState = UNKNOWN;
        this.enabled = enabled;

        if (enabled) {
            controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kExampleSubsystem);
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        controller.periodic();
        NinjasLogger.log("ExampleSubsystem/Controller", controller.getLogs());

        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.sequence(
            reset(),
            Commands.waitUntil(this::isReset)
        ));

        addEdge(RESET, IDLE);

        addEdge(IDLE, SHOOT, setVelocityCmd(PositionsConstants.ExampleSubsystem.kShoot.get()));
        addStateCommand(SHOOT, Commands.sequence(
            Commands.waitSeconds(1.5),
            setVelocityCmd(PositionsConstants.ExampleSubsystem.kShootFast.get())
        ));

        addEdge(SHOOT, IDLE, stopCmd());



        addStateEnd(RESET, () -> true, IDLE);

        addStateEnd(SHOOT, Seconds.of(3), IDLE);
    }

    public void setVelocity(double velocity) {
        if (!enabled)
            return;

        controller.setVelocity(velocity);
    }

    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public double getVelocity() {
        if (!enabled)
            return 0;

        return controller.getVelocity();
    }

    public boolean atGoal(){
        if (!enabled)
            return true;

        return controller.atGoal();
    }

    public Double getGoal() {
        if (!enabled)
            return 0.0;

        return controller.getGoal();
    }

    public void stop() {
        if (!enabled)
            return;

        controller.stop();
    }

    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(controller.getVelocity()) < 5;
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stopCmd();
    }
}