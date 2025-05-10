package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeCloseCommand extends CommandBase {
    private final Intake intake;

    public IntakeCloseCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeclose();
    }

    @Override
    public boolean isFinished() {
        return intake.IntakeServoData()[6] < 0.25;
    }
}
