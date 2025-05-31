package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Deposit;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Systems.Lifts;

public class SampleSwitchCommand extends CommandBase {
    private final Intake intake;
    private final Deposit deposit;
    public SampleSwitchCommand(Intake intake, Deposit deposit) {
        this.intake = intake;
        this.deposit  = deposit;
        addRequirements(intake, deposit);
    }

    @Override
    public void initialize() {
        deposit.setServoPosition(4, Constants.outtakeSampleDrop[3]);
        new WaitCommand(150);
       intake.setServoPosition(6,Constants.intakeActive[4]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
