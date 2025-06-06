package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;

public class IntakeGrabCommand extends SequentialCommandGroup {
    public final Intake intake;

    public IntakeGrabCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        addCommands(
                new InstantCommand(() -> { // Step 1: Set first batch of servos
                    intake.setServoPosition(1, Constants.intakeActive[0]);
                    intake.setServoPosition(2, Constants.intakeActive[1]);
                }),
                new WaitCommand(150),
                new InstantCommand(() -> intake.setServoPosition(6, Constants.intakeSampleReset[4]), intake),
                new WaitCommand(150),

                new InstantCommand(() -> {
                    intake.setServoPosition(1,0.6);
                    intake.setServoPosition(2,0.4);
                }, intake),
                new WaitCommand(200),

                new InstantCommand(() -> {
                    intake.setServoPosition(3,0.22);
                    intake.setServoPosition(4,0.78);
                }, intake)
        );
    }
}