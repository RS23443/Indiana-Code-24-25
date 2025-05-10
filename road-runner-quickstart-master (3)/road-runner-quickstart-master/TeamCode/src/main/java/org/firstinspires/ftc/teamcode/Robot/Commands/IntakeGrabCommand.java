package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeGrabCommand extends CommandBase {
    private final Intake intake;

    public IntakeGrabCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.teledropdowntograb();
    }

    @Override
    public boolean isFinished() {
        return intake.IntakeServoData()[6] < 0.25  && intake.IntakeServoData()[2] < 0.71;
    }
}
