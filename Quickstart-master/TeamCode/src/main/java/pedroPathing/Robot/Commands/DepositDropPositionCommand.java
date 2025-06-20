package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Deposit;


public class DepositDropPositionCommand extends SequentialCommandGroup {
    public final Deposit deposit;

    public DepositDropPositionCommand(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);

        addCommands(
                new InstantCommand(() -> deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]), deposit),
                new WaitCommand(150),
                new InstantCommand(() -> {
                    deposit.setServoPosition(1, Constants.outtakeSampleDrop[0]);
                    deposit.setServoPosition(2, Constants.outtakeSampleDrop[1]);
                    deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);
                }, deposit)
        );
    }
}