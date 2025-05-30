package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;

public class IntakeGrabCommand extends CommandBase {
    private final Intake intake;

    public IntakeGrabCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setServoPosition(5,Constants.intakeSampleReset[4]);
        new WaitCommand(150);
        intake.setServoPosition(1, Constants.intakeSampleReset[0]);
        intake.setServoPosition(2,Constants.intakeSampleReset[1]);
        intake.setServoPosition(3,Constants.intakeSampleReset[2]);
        intake.setServoPosition(4,Constants.intakeSampleReset[3]);
        new WaitCommand(200);
        intake.setMotor(0);

    }

    @Override
    public boolean isFinished() {
        return intake.getExtensionPosition() < 20;
    }
}
