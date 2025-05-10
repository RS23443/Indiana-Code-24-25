package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeClearanceTransferCommand extends CommandBase {
    private final Intake intake;

    public IntakeClearanceTransferCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.teleTransferStage1();
    }

    @Override
    public boolean isFinished() {
        return intake.IntakeServoData()[2] > 0.72 && intake.IntakeServoData()[4] > 0.9 && intake.IntakeServoData()[6] > 0.26;
    }
}
