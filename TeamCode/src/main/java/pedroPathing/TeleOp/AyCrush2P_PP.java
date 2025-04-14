package pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import pedroPathing.Crush;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@TeleOp(name = "AyCrush 2P PP - UIL", group = "UIL - ITD PP Teleop")
public class AyCrush2P_PP extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    //Crush Variables
    private enum IntakeState{
        IN,
        OUT
    }
    private enum SampleScoringState{
        INIT,
        INTAKING,
        AIMING,
        GRAB,
        TRANSFER,
        TRANSFERED,
        SCORING,
        SCORED

    }
    SampleScoringState sampleScoringState = SampleScoringState.INIT;
    private enum IntakeCurrState{
        IN,
        OUT
    }

    private DcMotorEx OuttakeSliderLeft;
    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private Servo IntakeClaw;

    private Servo OuttakeWrist;
    private Servo OuttakeElbowRight;
//    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    private Servo OuttakeElbow;
    private Servo IntakeElbowRight;
//    private Servo IntakeElbowLeft;
    private Servo IntakeWrist;
    public static int HIGH_BASKET = 4150;
    public static int HIGH_CHAMBER = 600;
    public static int initialPositionLeft, initialPositionRight;

    private boolean IntakeElbowDown = false;
    private boolean OuttakeElbowDown = false;
    private boolean OuttakeClawOpen = false;
    private boolean IntakeClawOpen = true;
    private boolean IntakeWristChanged = false;
    private boolean IntakeSliderChanged = false;
    final double IntakeClawPositionClose = 0.6;
    final double IntakeClawPositionOpen = 0.35;
    public static double IntakeSliderPositionOut = 0.6;
    final double IntakeSliderPositionIN = 0.4;
    final double IntakeElbowPositionIn = 0.25;
    final double IntakeElbowPositionOut = 0.77;
    final double IntakeElbowPositionGrab = 0.82;
    final double OuttakeElbowPositionIn = 0.33;
    final double OuttakeElbowPositionTransfer = 0.3;
    final double OuttakeElbowPositionSpecimenScoring = 0.21;
    final double OuttakeElbowPositionOut = 0.85;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeWristPositionOut = 0.80;
    final double OuttakeWristPositionIn = 0.00;
    final double OuttakeWristPositionTransfer = 0.79;
    final double OuttakeClawPositionClose = 0.35;
    final double OuttakeClawPositionOpen = 0.70;
    boolean TurnOuttakeSlidersOff = false;
    private int intakeCurrentState = 0;
    public int PlayerSelection = 1;
    public int leftPosition, rightPosition;
    public ElapsedTime slidersElapsedTime, transferTime, sampleScoringTime;



    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        //Pedro Pathing
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        /**
         * Intake Code Starts Here
         */
        //Intake Code
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        IntakeSliderLeft.scaleRange(0.0, 1.0);
        IntakeSliderRight.scaleRange(0.0, 1.0);

        //Servo Claw and Elbow Mapping and Setup
//        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
//        IntakeElbowLeft = hardwareMap.get(Servo.class, "IntakeElbowLeft");

        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);
//        IntakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        //IntakeClaw.scaleRange(0.0, 1.0);
        //IntakeElbowRight.scaleRange(0.0, 1.0);
        //IntakeElbowLeft.scaleRange(0.0, 1.0);

        IntakeWrist.setPosition(0.5); //Init Position for Wrist
        IntakeElbowRight.setPosition(IntakeElbowPositionIn); //Init Position for Wrist
        /**
         * Outtake Code Starts Here
         */
        //Outtake
        // Sliders Mapping and Setup
        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");
        OuttakeSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if(Crush.getInstance().areSlidersInitialized()){
//            initialPositionLeft = Crush.getInstance().getLeft();
            initialPositionRight = Crush.getInstance().getRight();

        }else{
            initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
//            initialPositionRight = OuttakeSliderRight.getCurrentPosition();
        }


        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
//        OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowRight.setDirection(Servo.Direction.REVERSE);

        TurnOuttakeSlidersOff = false;
        OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
        OuttakeWrist.setPosition(OuttakeWristPositionOut);
        OuttakeClaw.setPosition(OuttakeClawPositionOpen);

        slidersElapsedTime = new ElapsedTime();
        slidersElapsedTime.reset();

        transferTime = new ElapsedTime();


        transferTime.reset();
        telemetry.addData("Initial Pos Right", initialPositionRight);
        telemetry.addData("Initial Pos Left", initialPositionLeft);
    }
    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        /**Pedro Pathing Driving
         *
         */
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();


        switch (PlayerSelection){
            case 1:
//                PlayerSelection = 2;
                //All Intake Code
                //Intake Slider
                switch(sampleScoringState){
                    case INIT:

                                if(gamepad1.cross && intakeCurrentState==0){
                                    intakeSlidersElbow(IntakeState.OUT);
                                    intakeCurrentState=1;
//                                    IntakeSliderChanged = true;
                                    sampleScoringState = SampleScoringState.AIMING;
                                } else if (!gamepad1.cross) {
                                    intakeSlidersElbow(IntakeState.IN);
                                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                                    //OuttakeElbowMove(OuttakeElbowPositionMiddle);
                                    intakeCurrentState = 0;
                                }
                        break;
                    case INTAKING:

                        break;
                    case AIMING:
                        if(gamepad1.cross && intakeCurrentState==0){
                            IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
                            IntakeClaw.setPosition(IntakeClawPositionClose);
                            intakeCurrentState = 1;
                            sampleScoringState = SampleScoringState.GRAB;
                        } else if (!gamepad1.cross) {
                            intakeCurrentState=0;
                        }
                        break;
                    case GRAB:
                        if(gamepad1.cross && intakeCurrentState==0){
                            intakeSlidersElbow(IntakeState.IN);
                            intakeCurrentState=1;
                            sampleScoringState = SampleScoringState.TRANSFER;
                            transferTime.reset();
                        } else if (!gamepad1.cross) {
                            intakeCurrentState = 0;
                        }
                        break;
                    case TRANSFER:
                            if(transferTime.seconds()>1){

                                OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                                OuttakeElbowMove(OuttakeElbowPositionIn);
                            }
                        break;
                    case TRANSFERED:

                        break;
                    case SCORING:

                        break;
                    case SCORED:

                        break;
                }
                if(gamepad1.triangle && sampleScoringState != SampleScoringState.INIT){
                    if(sampleScoringState == SampleScoringState.GRAB){
                        sampleScoringState = SampleScoringState.AIMING;
                    }else{
                        sampleScoringState = SampleScoringState.INIT;
                    }

                }
                /*if(gamepad1.cross && !IntakeSliderChanged){
                    if(intakeCurrentState==0){
                        intakeSlidersElbow(IntakeState.OUT);
                        intakeCurrentState=1;
                    } else if (intakeCurrentState==1) {
                        intakeSlidersElbow(IntakeState.IN);
                        intakeCurrentState=0;
                    }
                    IntakeSliderChanged = true;
                } else if (!gamepad1.cross) {
                    IntakeSliderChanged = false;
                }*/
                if(gamepad1.left_trigger>0.25){
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                } else if (gamepad1.right_trigger>0.25) {
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                }

                if(gamepad1.ps){
                    OuttakeSlidersResetInitialPosition();
                }

                //All Outtake Code
                //Outtake Sliders Programming
                /*if(gamepad1.triangle){
                    outtakeSliders(HIGH_BASKET, 2000, 0);
                    //OuttakeElbowMove(OuttakeElbowPositionOut);
                }else*/ if(gamepad1.square){
                    outtakeSliders(HIGH_CHAMBER, 2000, 0);
                    OuttakeElbowMove(OuttakeElbowPositionSpecimenScoring);
                }else if(gamepad1.circle){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                    outtakeSliders(0, 2000, 0);

                }/*else if(gamepad1.dpad_down){

                    outtakeSliders(0, 2000, 0);

                }*/

                //Outtake elbow automatically goes out after reaching high basket position
                if((OuttakeSliderLeft.getCurrentPosition()>(HIGH_BASKET-20))
                        && (((OuttakeElbowRight.getPosition()==OuttakeElbowPositionMiddle))
                        || (OuttakeElbowRight.getPosition()==OuttakeElbowPositionIn))){

                    OuttakeElbowMove(OuttakeElbowPositionOut);
                }
                if(TurnOuttakeSlidersOff && OuttakeSliderLeft.getCurrentPosition()<=initialPositionLeft && slidersElapsedTime.seconds() > 5){
                    OuttakeSliderLeft.setPower(0.0);
                }

                //Outtake Elbow Middle
                /*if(gamepad1.dpad_up){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                } else if (gamepad1.dpad_left) {
                    OuttakeElbowMove(OuttakeElbowPositionIn);
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                } else if (gamepad1.dpad_right) {
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    OuttakeElbowMove(OuttakeElbowPositionOut);
                }*/

                //Outtake Claw Toggle
                if(gamepad1.left_bumper && !OuttakeClawOpen){
                    if(OuttakeClaw.getPosition() == OuttakeClawPositionOpen){
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    }else{
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    }
                    OuttakeClawOpen = true;
                }else if (!gamepad1.left_bumper) {
                    OuttakeClawOpen = false;
                }
                break;
            case 2:
                //All Intake Code (Player 1)
                //Intake Slider
               /* if(gamepad1.cross && !IntakeSliderChanged){
                    if(intakeCurrentState==0){
                        intakeSlidersElbow(IntakeState.OUT);
                        intakeCurrentState=1;
                    } else if (intakeCurrentState==1) {
                        intakeSlidersElbow(IntakeState.IN);
                        intakeCurrentState=0;
                    }
                    IntakeSliderChanged = true;
                } else if (!gamepad1.cross) {
                    IntakeSliderChanged = false;
                }*/
                 if(gamepad1.left_trigger>0.25){
                     IntakeClaw.setPosition(IntakeClawPositionClose);
                 } else if (gamepad1.right_trigger>0.25) {
                     IntakeClaw.setPosition(IntakeClawPositionOpen);
                 }
                 if(gamepad1.dpad_left){
                     IntakeSliderMove(0.005);
                 }else if(gamepad1.dpad_right){
                     IntakeSliderMove(-0.005);
                 }

                if(gamepad1.square && intakeCurrentState==1){
//                    IntakeElbowLeft.setPosition(IntakeElbowPositionGrab);
                    IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                }

                if(gamepad1.left_trigger>0.25){
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                } else if (gamepad1.right_trigger>0.25) {
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                }
                if (gamepad1.left_bumper && IntakeWrist.getPosition() <= 0.6 && !IntakeWristChanged){
                    IntakeWrist.setPosition(IntakeWrist.getPosition() + 0.075);
                    telemetry.addData("Intake Wrist Pos: ", IntakeWrist.getPosition());
                    IntakeWristChanged = true;
                }else if(gamepad1.right_bumper && IntakeWrist.getPosition() > 0.3 && !IntakeWristChanged){
                    IntakeWrist.setPosition(IntakeWrist.getPosition() - 0.075);
                    telemetry.addData("Intake Wrist Pos: ", IntakeWrist.getPosition());
                    IntakeWristChanged = true;
                } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    IntakeWristChanged = false;
                }


                //All Outtake Code (Player 2)
                //Outtake Sliders Programming
                if(gamepad2.triangle){
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    outtakeSliders(HIGH_BASKET, 2000, 0);
                    //OuttakeElbowMove(OuttakeElbowPositionOut);
                }else if(gamepad2.square){
                    outtakeSliders(HIGH_CHAMBER, 2000, 0);
                }else if(gamepad2.cross){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                    outtakeSliders(5, 2000, 0);

                }else if(gamepad2.dpad_down){

                    outtakeSliders(-100, 2000, 1);

                }else if(gamepad2.dpad_up){

                    outtakeSliders(100, 2000, 1);

                }


                //Outtake Elbow Middle
                if(gamepad2.left_bumper){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                } else if (gamepad2.dpad_left) {
                    OuttakeElbowMove(OuttakeElbowPositionOut);
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                } else if (gamepad2.dpad_right) {
                    //OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    //OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    OuttakeElbowMove(OuttakeElbowPositionIn);
                }else if (gamepad2.right_bumper) {
                    //OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    //OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeElbowMove(OuttakeElbowPositionSpecimenScoring);
                }

                //Outtake Claw Toggle
                /*if(gamepad1.left_bumper && !OuttakeClawOpen){
                    if(OuttakeClaw.getPosition() == OuttakeClawPositionOpen){
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    }else{
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    }
                    OuttakeClawOpen = true;
                }else if (!gamepad1.left_bumper) {
                    OuttakeClawOpen = false;
                }*/
                if(gamepad2.left_trigger > .25){
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                }else if(gamepad2.right_trigger>0.25){
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                }
                if(gamepad2.ps){
                    OuttakeSlidersResetInitialPosition();
                }else if(gamepad2.circle){
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    outtakeSliders(HIGH_CHAMBER,2000,0);
                }
                break;
            default:

                break;

        }

        //General code for both options (1 or 2 Players)
        if((TurnOuttakeSlidersOff && slidersElapsedTime.seconds() > 5.0)){
            OuttakeSliderLeft.setPower(0.0);
            //OuttakeSliderRight.setPower(0.0);
            TurnOuttakeSlidersOff = false;
        }


        //Outtake elbow automatically goes out after reaching high basket position
        if((OuttakeSliderLeft.getCurrentPosition()>(HIGH_BASKET-20))
                && (((OuttakeElbowRight.getPosition()==OuttakeElbowPositionMiddle)
                || (OuttakeElbowRight.getPosition()==OuttakeElbowPositionIn)))){

            OuttakeElbowMove(OuttakeElbowPositionOut);
        }
        if(gamepad1.dpad_up){
            OuttakeWrist.setPosition(OuttakeWrist.getPosition() + 0.01);
        }else if(gamepad1.dpad_down){
            OuttakeWrist.setPosition(OuttakeWrist.getPosition() - 0.01);
        }
        if(gamepad1.share){
            PlayerSelection = 2;
        }else if(gamepad1.options){
            PlayerSelection = 1;
        }
        if(gamepad1.dpad_left){
            OuttakeElbowRight.setPosition(OuttakeElbowRight.getPosition() + 0.01);
        }else if(gamepad1.dpad_right){
            OuttakeElbowRight.setPosition(OuttakeElbowRight.getPosition() - 0.01);
        }
        if(gamepad1.share){
            PlayerSelection = 2;
        }else if(gamepad1.options){
            PlayerSelection = 1;
        }

        //Yellow = red: 1400-1500; green: 1900-2000; blue:450-550;
        //Blue = red: 250 - 350; green: 600 - 700; blue: 1450 - 1550;
        //Red = red: 1350 - 1450; green: 750 - 850; blue: 350 - 450;
        //All Telemetry goes here
        // Lets get the target positions.
        telemetry.addData("Intake Slider Right Pos: ", IntakeSliderRight.getPosition() );
        telemetry.addData("Intake Slider Left Pos: ", IntakeSliderLeft.getPosition() );
//        telemetry.addData("OuttakeRight Pos: ", OuttakeSliderRight.getCurrentPosition() );
        telemetry.addData("OuttakeLeft Pos: ", OuttakeSliderLeft.getCurrentPosition() );
//        telemetry.addData("Outtake Elbow L Pos: ", OuttakeElbowLeft.getPosition() );
        telemetry.addData("Outtake Elbow R Pos: ", OuttakeElbowRight.getPosition() );
        telemetry.addData("Outtake Elbow Down: ", OuttakeElbowDown );
        telemetry.addData("Intake Elbow Position: ", IntakeElbowRight.getPosition() );
        telemetry.addData("Outtake Wrist Position: ", OuttakeWrist.getPosition() );
//        telemetry.addData("Intake Claw Open: ", IntakeClawOpen );
//        telemetry.addData("Right Slider: ", OuttakeSliderRight.getPower() );
        telemetry.addData("Left Slider: ", OuttakeSliderLeft.getPower() );
        telemetry.addData("Left Position: ", leftPosition );
        telemetry.addData("Right Position: ", rightPosition );
        telemetry.addData("CURRENT STATE: ", intakeCurrentState );
        /*telemetry.addData("intake Sensor red: ", IntakeSensor.red() );
        telemetry.addData("intake Sensor green: ", IntakeSensor.green() );
        telemetry.addData("intake Sensor blue: ", IntakeSensor.blue() );*/




    }
    public void outtakeSliders(int targetPosition, int velocity, int manual_override){
        // Set run modes for both motors
        //OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Synchronize motion
        if(manual_override == 0) {
            //OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
            OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);
        }else{
            //OuttakeSliderRight.setTargetPosition(targetPosition + OuttakeSliderRight.getCurrentPosition());
            OuttakeSliderLeft.setTargetPosition(targetPosition + OuttakeSliderLeft.getCurrentPosition());
        }
        if(targetPosition < 10){
            TurnOuttakeSlidersOff = true;
            slidersElapsedTime.reset();
        }
        //Run to target position
        //OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power
        //OuttakeSliderRight.setPower(0.95);
        OuttakeSliderLeft.setPower(1.0);
        // while(OuttakeSliderRight.isBusy() && OuttakeSliderLeft.isBusy()){

        //}

        /*
        // Set the same PID coefficients for both motors
        OuttakeSliderRight.setVelocityPIDFCoefficients(2.0, 0.5, 0.1, 0.0);
        OuttakeSliderLeft.setVelocityPIDFCoefficients(2.0, 0.5, 0.1, 0.0);

        // Synchronize motion
        OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
        OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);

        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        OuttakeSliderRight.setVelocity(velocity);  // Set velocity in ticks per second
        OuttakeSliderLeft.setVelocity(velocity);
        */
    }
    public void OuttakeSlidersResetInitialPosition(){
        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        //initialPositionRight = OuttakeSliderRight.getCurrentPosition();
        //OuttakeSliderRight.setPower(0.0);
        OuttakeSliderLeft.setPower(0.0);
    }
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
                IntakeElbowRight.setPosition(IntakeElbowPositionIn);
//                IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                IntakeWrist.setPosition(IntakeWristInitialPosition);

                break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                IntakeElbowRight.setPosition(IntakeElbowPositionOut);
//                IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                IntakeClaw.setPosition(IntakeClawPositionOpen);

                break;

        }

    }
    private void IntakeSliderMove(double position){
            IntakeSliderLeft.setPosition(IntakeSliderLeft.getPosition() + position);
            IntakeSliderRight.setPosition(IntakeSliderRight.getPosition() + position);

    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        //OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }

    private int getColor(int r, int g, int b){
        int color = 0;
        if((r>2700 && r <3200) && (g>3500 && g<4200) && (b>500 && b<1200)){
            color = 1; //Yellow
        }else if((r>200 && r <500) && (g>400 && g<700) && (b>1000 && b<1500)){
            color = 2; //Blue
        }else if((r>2200 && r <2600) && (g>1000 && g<1500) && (b>500 && b<800)){
            color = 3; //Red
        }else{
            color = 0; //other color
        }
        return color;
    }
    public void setSampleScoringState(SampleScoringState pState) {
        sampleScoringState = pState;
        transferTime.reset();

    }

}