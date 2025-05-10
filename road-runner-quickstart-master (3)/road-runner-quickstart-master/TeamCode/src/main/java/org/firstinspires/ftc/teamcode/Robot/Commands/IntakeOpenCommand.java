package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeOpenCommand extends CommandBase {
    private final Intake intake;

    public IntakeOpenCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeopen();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
