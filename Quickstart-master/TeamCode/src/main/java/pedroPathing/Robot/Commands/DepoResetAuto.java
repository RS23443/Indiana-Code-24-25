package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Deposit;


public class DepoResetAuto extends SequentialCommandGroup {
    public final Deposit deposit;

    public DepoResetAuto(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);

        addCommands(
                new InstantCommand(() -> deposit.setServoPosition(4, Constants.outtakeSampleReset[3]), deposit),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    deposit.setServoPosition(1, 0.48);
                    deposit.setServoPosition(2, 0.52);
                    deposit.setServoPosition(3, 1);
                }, deposit)
        );
    }
}