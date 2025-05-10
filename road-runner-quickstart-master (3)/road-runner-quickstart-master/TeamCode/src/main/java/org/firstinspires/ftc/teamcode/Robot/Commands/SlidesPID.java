package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;

public class SlidesPID extends CommandBase {
    private final Lifts liftSubsystem;
    private final int targetPosition;

    /**
     * Constructor for the MoveSlideCommand
     *
     * @param liftSubsystem  The subsystem controlling the slides
     * @param targetPosition The target position in ticks
     */
    public SlidesPID(Lifts liftSubsystem, int targetPosition) {
        this.liftSubsystem = liftSubsystem;
        this.targetPosition = targetPosition;

        addRequirements(liftSubsystem); // Declare subsystem dependency
    }

    @Override
    public void initialize() {
        // Set the target position using the lift subsystem's PID control
        liftSubsystem.slidePID(targetPosition);
    }

    @Override
    public void execute() {
        // Continuously update the slide movement using PID
        liftSubsystem.slidePID(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Check if the slides have reached the target position
        return liftSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the slides when the command ends
        liftSubsystem.stop();
    }
}
