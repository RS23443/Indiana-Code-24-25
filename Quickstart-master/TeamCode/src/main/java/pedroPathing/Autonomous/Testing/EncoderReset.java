package pedroPathing.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.Robot.Systems.Intake;
import pedroPathing.Robot.Systems.Lifts;


@TeleOp
public class EncoderReset extends OpMode {
    public Lifts lifts;
    public Intake intake;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        lifts = new Lifts(hardwareMap,12.26);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        intake.resetExtension();
        lifts.stopandreset();
    }

    @Override
    public void loop() {
        int mslidepos = (int) lifts.getMiddleMotorData()[0];
        int bslidepos = (int) lifts.getMiddleMotorData()[0];
        int tslidepos = (int) lifts.getTopMotorData()[0];
        int intakepos =  intake.getExtensionPosition();

        telemetry.addData("intake Position", intakepos);
        telemetry.addData("top pos", tslidepos);
        telemetry.addData("mis pos", mslidepos);
        telemetry.addData("bottom pos", bslidepos );
    }
}
