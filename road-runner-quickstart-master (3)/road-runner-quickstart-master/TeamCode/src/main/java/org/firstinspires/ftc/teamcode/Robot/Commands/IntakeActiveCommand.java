package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;


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
            intake.setServoPosition(2, Constants.intakeActive[1]);
            new WaitCommand(200);
            intake.setServoPosition(3, Constants.intakeActive[2]);
            intake.setServoPosition(4, Constants.intakeActive[3]);
            new WaitCommand(200);
            intake.setServoPosition(6, Constants.intakeActive[4]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
