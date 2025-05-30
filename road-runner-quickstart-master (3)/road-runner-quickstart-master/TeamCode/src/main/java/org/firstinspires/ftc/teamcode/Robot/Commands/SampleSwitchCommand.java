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
    private final Lifts lifts;

    public SampleSwitchCommand(Intake intake, Deposit deposit, Lifts lifts) {
        this.intake = intake;
        this.deposit  = deposit;
        this.lifts = lifts;
        addRequirements(intake, deposit, lifts);
    }

    @Override
    public void initialize() {
        new PDFLCommand(lifts,20);
        new WaitCommand(150);
        deposit.setServoPosition(4,Constants.outtakeSpecimenDrop[3]);
        new WaitCommand(150);
        intake.setServoPosition(5,Constants.intakeActive[4]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
