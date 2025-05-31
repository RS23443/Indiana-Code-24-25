package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;

public class IntakeActiveCommand extends SequentialCommandGroup {
    public final Intake intake;
    public IntakeActiveCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        addCommands(
                new InstantCommand(() -> {
                    intake.setServoPosition(1, Constants.intakeActive[0]);
                    intake.setServoPosition(2, Constants.intakeActive[1]);
                }, intake),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    intake.setServoPosition(3, Constants.intakeActive[2]);
                    intake.setServoPosition(4, Constants.intakeActive[3]);
                }, intake),
                new WaitCommand(200),
                new InstantCommand(() -> intake.setServoPosition(6, Constants.intakeActive[4]), intake)
        );
    }
}