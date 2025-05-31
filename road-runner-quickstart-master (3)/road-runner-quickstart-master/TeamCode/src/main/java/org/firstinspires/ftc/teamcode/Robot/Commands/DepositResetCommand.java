package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;


public class DepositResetCommand extends SequentialCommandGroup {
    public final Deposit deposit;

    public DepositResetCommand(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);

        addCommands(
                new InstantCommand(() -> deposit.setServoPosition(4, Constants.outtakeSampleReset[3]), deposit),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    deposit.setServoPosition(1, Constants.outtakeSampleReset[0]);
                    deposit.setServoPosition(2, Constants.outtakeSampleReset[1]);
                    deposit.setServoPosition(3, Constants.outtakeSampleReset[2]);
                }, deposit)
        );
    }
}