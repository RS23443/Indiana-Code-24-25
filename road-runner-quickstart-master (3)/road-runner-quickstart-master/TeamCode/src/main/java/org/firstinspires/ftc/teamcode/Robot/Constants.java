package org.firstinspires.ftc.teamcode.Robot;

public final class Constants {
    private Constants(){
        //restrict instantiations
    }
    public static final double[] outtakeSampleReset = {0.5,0.5,0.98,0.8};
    /*
    This is will cause the outtake arm to go into grabbing position
    left_outtake_flip = outtakeSampleReset[0]
    right_outtake_flip = outtakeSampleReset[1]
    right_elbow = outtakeSampleReset[2]
    outtake_claw = outtakeSampleReset[3]
    the final number is the PDFL value for the lifts outtakeSampleReset[4]
    */

    public static final double[] outtakeSampleDrop = {0.7,0.3,0.35,0.63};
    /*
    this is will cause the outtake arm to grab and flip outwards ready to deposit Sample
    left_outtake_flip = outtakeSampleDrop[0]
    right_outtake_flip = outtakeSampleDrop[1]
    right_elbow = outtakeSampleDrop[2]
    outtake_claw = outtakeSampleDrop[3]
    the final number is the PDFL value for the lifts outtakeSampleDrop[4]
     */
    public static final double[] outtakeSpecimenReset = {0.0,0.0,0.0,0.0};
    /*
    this is will cause the outtake arm to grab and flip outwards ready to deposit Sample
    left_outtake_flip = outtakeSpecimenReset[0]
    right_outtake_flip = outtakeSpecimenReset[1]
    right_elbow = outtakeSpecimenReset[2]
    outtake_claw = outtakeSpecimenReset[3]
    the final number is the PDFL value for the lifts outtakeSpecimenReset[4]
     */

    public static final double[] outtakeSpecimenDrop = {0.0,0.0,0.0,0.0};
    /*
    this is will cause the outtake arm to grab and flip outwards ready to deposit Sample
    left_outtake_flip = outtakeSpecimenDrop[0]
    right_outtake_flip = outtakeSpecimenDrop[1]
    right_elbow = outtakeSpecimenDrop[2]
    outtake_claw = outtakeSpecimenDrop[3]
    the final number is the PDFL value for the lifts, outtakeSpecimenDrop[4]
     */

    public static final double[] intakeActive = {0.88,0.12,0.5,0.5,0.7};
     /*
     This is will cause the intake to extend out and be primed to grab a block
    left_intake_flip = intakeSampleReset[0]
    right_intake_flip = intakeSampleReset[1]
    left_differential = intakeSampleReset[2]
    right_differential = intakeSampleReset[3]
    intake_claw = intakeSampleReset[4] -> would need to be open
    intake_extension = intakeSampleReset[5] - PDFL value
     */

    public static final double[] intakeSampleReset = {0.6,0.4,0.25,75,0.42};
     /*
    this will cause the intake to reel in the intake and have it ready for the outtake to grab the block
    left_intake_flip = intakeSampleReset[0]
    right_intake_flip = intakeSampleReset[1]
    left_differential = intakeSampleReset[2]
    right_differential = intakeSampleReset[3]
    intake_claw = intakeSampleReset[4] -> would need to be closed
    intake_extension = intakeSampleReset[5] - PDFL value

     */

    public static final double[] intakeSpecimenReset = {0.0,0.0,0.0,0.0,0.0};
     /*
    this will cause the intake to reel in the intake and have it ready to drop the block
    left_intake_flip = intakeSpecimenReset[0]
    right_intake_flip = intakeSpecimenReset[1]
    left_differential = intakeSpecimenReset[2]
    right_differential = intakeSpecimenReset[3]
    intake_claw = intakeSpecimenReset[4] -> would need to be closed
    intake_extension = intakeSpecimenReset[5] - PDFL value
     */

    public static final double[] intakeHorizontalSpin = {0.18,0.18, 0.35, 0.35};
    /*
    These constants are focused on spinning horizontally as everything else keeps a vertical spins
    left_differential = intakeHorizontalSpin[2]
    right_differential = intakeHorizontalSpin[3]
     */

    public static final int[] intakeExtensionValues = {0,600,150,100,0};
    /*
    these values are constants for maximum and minimum extension
    minimum_value = intakeExtensionValues[0]
    maximum_value = intakeExtensionValues[1]
    incremental_scans = intakeExtensionValues[2]
    retract value after finding yellow = intakeExtensionValue[3]
    stand_still velocity = intakeExtensionValue[4]
     */

    public static final int[] liftExtensionValues = {0,900,600,1500};
    /*
    these values are constants for lift position throughout the game
    minimum-value = liftExtensionValues[0]
    specimen high chamber = liftExtensionValues[1]
    low basket = liftExtensionValues[2]
    high basket = liftExtensionValues[3]
     */

    public static final double turnOffset = 22;
    public static final double linearOffset = 1.5;
    public static final double lateralOffset = 3.5;

}
