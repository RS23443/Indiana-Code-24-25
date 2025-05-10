package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeBringInCommand extends CommandBase {
    private final Intake intake;

    public IntakeBringInCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
//here is a comment that has no purpose
    @Override
    public void initialize() {
        intake.bringinintake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
