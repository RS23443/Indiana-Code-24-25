package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;
import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;

public class DepositResetCommand extends CommandBase {
    private final Deposit deposit;
    private final Lifts lifts;

    public DepositResetCommand( Deposit deposit, Lifts lifts) {
        this.deposit  = deposit;
        this.lifts = lifts;
        addRequirements( deposit, lifts);
    }

    @Override
    public void initialize() {
        deposit.setServoPosition(4, Constants.outtakeSampleReset[3]);// need to test
        new WaitCommand(200);
        deposit.setServoPosition(1, Constants.outtakeSampleReset[0]);
        deposit.setServoPosition(2, Constants.outtakeSampleReset[1]);
        deposit.setServoPosition(3, Constants.outtakeSampleReset[2]);
        new WaitCommand(200);
        new PDFLCommand(lifts,20);

    }

    @Override
    public boolean isFinished() {
        return lifts.getTopMotorData()[1] == 0;
    }
}
