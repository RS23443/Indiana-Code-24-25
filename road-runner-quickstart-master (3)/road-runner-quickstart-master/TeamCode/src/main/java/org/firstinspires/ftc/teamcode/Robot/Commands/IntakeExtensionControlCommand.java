package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;

public class IntakeExtensionControlCommand extends CommandBase {
    private final Intake intake;
    private final int target;
    public int initalPosition;
    public int direction;

    public IntakeExtensionControlCommand (Intake intake, int target) {
        this.intake = intake;
        this.target = target;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        initalPosition = intake.getExtensionPosition();
        direction = target - initalPosition;
        intake.setMotor(target);

    }

    @Override
    public void execute() {
        // Continuously update the slide movement using PID
        intake.setMotor(target);
    }

    @Override
    public boolean isFinished() {
        if(direction > 0) {
            return intake.getExtensionPosition() > target - 20;
        } else{
            return  intake.getExtensionPosition() < target + 20;
        }
    }
    @Override
    public void end(boolean interrupted) {
        if(direction < 0){
            intake.setMotorPower(0);
        } else {
            intake.setMotorPower(0.075);
        }
    }
}
