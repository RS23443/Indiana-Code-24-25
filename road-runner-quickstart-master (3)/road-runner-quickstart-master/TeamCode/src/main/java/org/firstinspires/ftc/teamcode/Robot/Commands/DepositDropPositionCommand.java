package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;

public class DepositDropPositionCommand extends CommandBase {
    private final Deposit deposit;

    public DepositDropPositionCommand(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]);
        new WaitCommand(150);
        deposit.setServoPosition(1, Constants.outtakeSampleDrop[0]);
        deposit.setServoPosition(2, Constants.outtakeSampleDrop[1]);
        deposit.setServoPosition(3, Constants.outtakeSampleDrop[2]);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
