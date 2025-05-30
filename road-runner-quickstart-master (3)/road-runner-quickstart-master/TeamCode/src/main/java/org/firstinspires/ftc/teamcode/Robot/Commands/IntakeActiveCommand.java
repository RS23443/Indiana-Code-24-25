package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;

public class IntakeActiveCommand extends CommandBase {
    private final Intake intake;

    public IntakeActiveCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setServoPosition(1, Constants.intakeActive[0]);
        intake.setServoPosition(2,Constants.intakeActive[1]);
        intake.setServoPosition(3,Constants.intakeActive[2]);
        intake.setServoPosition(4,Constants.intakeActive[3]);
        intake.setServoPosition(5,Constants.intakeActive[4]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
