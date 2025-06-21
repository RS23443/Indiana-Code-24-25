package pedroPathing.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.Robot.Constants;

@TeleOp
@Config
public class ServoPositionForSpec extends OpMode {
    private Servo lif;
    private Servo rif;
    private Servo rightdiffy;
    private Servo leftdiffy;
    private Servo claw;
    private Servo lof;
    private Servo rof;
    private Servo right_elbow;
    private Servo o_claw;
    @Override
    public void init() {
        lif = hardwareMap.get(Servo.class, "left_intake_flip");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        leftdiffy = hardwareMap.get(Servo.class, "left_differential");
        rightdiffy = hardwareMap.get(Servo.class, "right_differential");
        claw = hardwareMap.get(Servo.class, "intake_claw");
        lof = hardwareMap.get(Servo.class, "left_outtake_flip");
        rof = hardwareMap.get(Servo.class, "right_outtake_flip");
        right_elbow = hardwareMap.get(Servo.class, "right_elbow");
        o_claw = hardwareMap.get(Servo.class, "outtake_claw");

        telemetry = FtcDashboard.getInstance().getTelemetry();

        o_claw.setPosition(Constants.outtakeSampleDrop[3]);

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            lof.setPosition(0.5);
            rof.setPosition(0.5);
            right_elbow.setPosition(0.1);
        }
        if(gamepad1.b){
            lof.setPosition(0.6);
            rof.setPosition(0.4);
            right_elbow.setPosition(0.0);

        }

        if(gamepad1.x){
            right_elbow.setPosition(0.2);

        }

        if(gamepad1.right_bumper){
            right_elbow.setPosition(0.05);
        }

        if(gamepad1.y){
            right_elbow.setPosition(0.15);

        }

        if(gamepad2.a){
            lof.setPosition(0.7);
            rof.setPosition(0.3);
            right_elbow.setPosition(0.5);
        }

        if(gamepad2.b){
            lof.setPosition(0.75);
            rof.setPosition(0.25);
            right_elbow.setPosition(0.35);;
        }

        if(gamepad2.x){
            lof.setPosition(0.8);
            rof.setPosition(0.2);
            right_elbow.setPosition(0.35);
        }

        if(gamepad2.y){
            lof.setPosition(0.85);
            rof.setPosition(0.15);
            right_elbow.setPosition(0.4);
        }

        if(gamepad2.right_bumper){
            lof.setPosition(0.9);
            rof.setPosition(0.1);
            right_elbow.setPosition(0.35);
        }

        if(gamepad2.left_bumper){
            lof.setPosition(0.5);
            rof.setPosition(0.5);
            right_elbow.setPosition(0.1);
        }









        double lifpos = lif.getPosition();
        double rifpos = rif.getPosition();
        double ldpos = leftdiffy.getPosition();
        double rdpos = rightdiffy.getPosition();
        double icpos = claw.getPosition();
        double lofpos = lof.getPosition();
        double rofpos = rof.getPosition();
        double elbowpos = right_elbow.getPosition();
        double ocpos =o_claw.getPosition();

        telemetry.addData("lif pos", lifpos);
        telemetry.addData("rif pos", rifpos);
        telemetry.addData("leftdiffy pos", ldpos);
        telemetry.addData("rightdiffy pos", rdpos);
        telemetry.addData("intake claw pos", icpos);
        telemetry.addData("lof pos", lofpos);
        telemetry.addData("rof pos", rofpos);
        telemetry.addData("outtake elbow pos", elbowpos);
        telemetry.addData("outtake claw pos", ocpos);
    }
}
