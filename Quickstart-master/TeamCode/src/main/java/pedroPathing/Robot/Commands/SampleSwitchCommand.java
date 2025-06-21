package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;
import pedroPathing.Robot.Systems.Deposit;
// Lifts subsystem was removed from constructor in user's provided code, so not included here.
// If Lifts is still needed, add 'Lifts lifts' to constructor and 'lifts' to addRequirements.

public class SampleSwitchCommand extends SequentialCommandGroup {
    public final Intake intake;
    public final Deposit deposit;

    public SampleSwitchCommand(Intake intake, Deposit deposit) {
        this.intake = intake;
        this.deposit = deposit;
        addRequirements(intake, deposit);

        addCommands(
                new InstantCommand(() -> deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]), deposit),
                new WaitCommand(150),
                new InstantCommand(() -> intake.setServoPosition(6, Constants.intakeActive[4]), intake)
        );
    }
}