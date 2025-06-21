package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;

public class IntakeHorizontalSpinCommand extends CommandBase {
    private final Intake intake;

    public IntakeHorizontalSpinCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setServoPosition(3, Constants.intakeHorizontalSpin[0]);
        intake.setServoPosition(4, Constants.intakeHorizontalSpin[1]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
