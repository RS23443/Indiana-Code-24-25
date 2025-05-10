package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeExtendCommand extends CommandBase {
    private final Intake intake;

    public IntakeExtendCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.dropdownintake();
    }

    @Override
    public boolean isFinished() {
        return intake.IntakeServoData()[2] > 0.62 && intake.IntakeServoData()[2] > 0.72;
    }
}
