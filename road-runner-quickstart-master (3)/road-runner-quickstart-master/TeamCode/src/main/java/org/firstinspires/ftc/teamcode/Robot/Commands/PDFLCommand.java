package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;

public class PDFLCommand extends CommandBase {
    private final Lifts liftSubsystem;
    private final int targetPosition;

    /**
     * Constructor for the MoveSlideCommand
     *
     * @param liftSubsystem  The subsystem controlling the slides
     * @param targetPosition The target position in ticks
     */
    public PDFLCommand(Lifts liftSubsystem, int targetPosition) {
        this.liftSubsystem = liftSubsystem;
        this.targetPosition = targetPosition;

        addRequirements(liftSubsystem); // Declare subsystem dependency
    }

    @Override
    public void initialize() {
        // Set the target position using the lift subsystem's PID control
        liftSubsystem.setTarget(targetPosition);
        liftSubsystem.enablePDControl(true);
        liftSubsystem.slidePDFL(targetPosition);
    }

    @Override
    public void execute() {
        // Continuously update the slide movement using PID
        liftSubsystem.updatePDControl();
    }

    @Override
    public boolean isFinished() {
        // Check if the slides have reached the target position
        return (Math.abs(liftSubsystem.getTopMotorData()[0] - targetPosition) < 20)&&(liftSubsystem.getMiddleMotorData()[1] == 0 || liftSubsystem.getBottomMotorData()[1] == 0 || liftSubsystem.getMiddleMotorData()[1] == 0)  ;
    }
//Math.abs(liftSubsystem.getMiddleMotorData()[0] - targetPosition) < 20|| Math.abs(liftSubsystem.getBottomMotorData()[0] - targetPosition) < 20 ||
    @Override
    public void end(boolean interrupted) {
        // Stop the slides when the command ends
        liftSubsystem.runslides(liftSubsystem.DynamicKF());
    }
}
