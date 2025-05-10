package pedroPathing.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.Crush;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */
@Config
@Autonomous(name = "ITD_UIL_Left-2", group = "Auto")
public class ITD_UIL_Left2 extends OpMode {

    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeWrist;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 2500;
    final int HIGH_CHAMBER = 2200;
    public int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionOut = 0.;
    final double OuttakeElbowPositionSpecimenScoring = 0.30;

    final double OuttakeElbowPositionIn = 0.33;
    final double OuttakeElbowPositionTransfer = 0.27;
    final double OuttakeElbowPositionMiddle = 0.5;
    final double OuttakeWristPositionInit = 0.2;
    final double OuttakeWristPositionTransfer = 0.15;
    final double OuttakeWristPositionAiming = 0.5;
    final double OuttakeWristPositionScoring = 0.75;
    final double OuttakeWristPositionScoreBasket = 0.75;
    final double OuttakeClawPositionClose = 0;
    final double OuttakeClawPositionOpen = 1.0;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private Servo IntakeWrist;
    final double IntakeWristInitialPosition = 0.5;
    private Servo IntakeElbowRight;


    private enum IntakeState{
        IN,
        OUT
    }
    final double IntakeClawPositionClose = 0.6;
    final double IntakeClawPositionOpen = 0.35;
    final double IntakeSliderPositionOut = 0.6;
    final double IntakeSliderPositionIN = 0.4;
    final double IntakeElbowPositionIn = 0.2;
    final double IntakeElbowPositionOut = 0.77;
    final double IntakeElbowPositionGrab = 0.85;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int initialPos;
    private Servo IntakeClaw;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 112, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    public static double scorePoseX = 14.25;
    public static double scorePoseY = 128.0;
    private final Pose highBasketPose = new Pose(scorePoseX, scorePoseY, Math.toRadians(315));
    private final Pose highBasketPose2 = new Pose(scorePoseX, 118, Math.toRadians(315));
    private final Pose highBasketPose3 = new Pose(scorePoseX, scorePoseY, Math.toRadians(315));
    private final Pose highBasketPose4 = new Pose(scorePoseX, scorePoseY, Math.toRadians(315));
    public static double hbControlX = 27.0;
    public static double hbControlY = 118.0;
    private final Pose controlHighBasketPose = new Pose (hbControlX,hbControlY, Math.toRadians(0));
    /** Lowest (First) Sample from the Spike Mark */
    public static double samplePos1X = 21.9;
    public static double samplepos1Y = 124;
    private final Pose samplepos1 = new Pose(samplePos1X, samplepos1Y, Math.toRadians(0));
    private final Pose controlsamplepos1 = new Pose(80, 36, Math.toRadians(0));
    public static double samplePos3X = 42.75;
    public static double samplepos3Y = 125;
    private final Pose samplepos3 = new Pose(samplePos3X, samplepos3Y, Math.toRadians(90));

    public static double samplePos2X = 20.5;
    public static double samplepos2Y = 132;
    private final Pose samplepos2 = new Pose(samplePos2X, samplepos2Y, Math.toRadians(0));
    private final Pose controlsamplepos2 = new Pose(75, 26, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */

    public static double samplepush2X = 19.0;
    public static double samplepush2Y = 17.0;
    private final Pose samplepush2 = new Pose(samplepush2X, samplepush2Y, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup = new Pose(27, 30, Math.toRadians(0));
    private final Pose pickup2 = new Pose(18, 30, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(10, 20, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(11, 11, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain sample1, sample1Score, sample2, sample2Score, sample3, sample3Score;
    private boolean range;
    public void sliderMove (int Position){
        OuttakeSliderLeft.setTargetPosition(initialPositionLeft + Position);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setPower(.78);
        /*while (OuttakeSliderLeft.isBusy() && OuttakeSliderRight.isBusy()){

        }
        OuttakeSliderLeft.setPower(0);
        OuttakeSliderRight.setPower(0);*/
    }


    public void outtakeElbow (double SPosition){
        OuttakeElbowRight.setPosition(SPosition);
    }
    public void rangeXY (int X,int Y) {
        range = false;
        if (follower.getPose().getX() >= X && follower.getPose().getX() < X + 1 && follower.getPose().getY() >= Y && follower.getPose().getY() < Y + 1) {
            range = true;
        }
    }



    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/


    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(controlHighBasketPose), new Point(highBasketPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(),highBasketPose.getHeading());

        /* Gets in Position to push the 1st Sample into the Observation Zone */
        sample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose),new Point(samplepos1)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), samplepos1.getHeading())
                .build();
        sample1Score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepos1),new Point(highBasketPose2)))
                .setLinearHeadingInterpolation(samplepos1.getHeading(), highBasketPose2.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        sample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose2),new Point(samplepos2)))
                .setLinearHeadingInterpolation(highBasketPose2.getHeading(), samplepos2.getHeading())
                .build();
        sample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose2),new Point(samplepos3)))
                .setLinearHeadingInterpolation(highBasketPose2.getHeading(), samplepos3.getHeading())
                .build();
        sample3Score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepos3),new Point(scorePoseX,scorePoseY + 1)))
                .setLinearHeadingInterpolation(samplepos3.getHeading(), highBasketPose3.getHeading())
                .build();
        sample2Score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepos2),new Point(scorePoseX,scorePoseY + 1)))
                .setLinearHeadingInterpolation(samplepos2.getHeading(), highBasketPose4.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(highBasketPose4), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(highBasketPose4.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
            ///Aims High Basket
                follower.followPath(scorePreload, true);
                //OuttakeElbowRight.setPosition(0.5);
                setPathState(1);
                break;

            case 1:
            ///Prepares Outtake Elbows for scoring
                if (follower.getPose().getX() >= startPose.getX() && follower.getPose().getY() >= startPose.getY()) {
                    sliderMove(HIGH_BASKET);
                    OuttakeWrist.setPosition(OuttakeWristPositionAiming);

                    setPathState(2);
                }
                break;
            case 2:
            ///Aims Preload with Elbow
                if (pathTimer.getElapsedTimeSeconds() >= 2.25) {
                    //
                    //
                    OuttakeWrist.setPosition(OuttakeWristPositionScoreBasket);

                    setPathState(3);
                }
                break;
            case 3:
            ///Scores Preload
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    if (pathTimer.getElapsedTimeSeconds() >= 1) {


                        setPathState(4);
                    }
                }
                break;
            case 4:
            ///Move Sliders Down and Start PAth towards 1st Sample
                if (pathTimer.getElapsedTimeSeconds() >= 0.05) {
                    follower.followPath(sample1);
                    OuttakeWrist.setPosition(OuttakeWristPositionInit);
                    sliderMove(0);
                    if (pathTimer.getElapsedTimeSeconds() >= 1.15){
                        OuttakeWrist.setPosition(OuttakeWristPositionInit);
                        OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                        IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                        IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                        IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                        IntakeClaw.setPosition(IntakeClawPositionOpen);
                        setPathState(5);
                }
                }
                break;
            case 5:
            ///Grabs 1st Sample from the floor
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5) {
                    IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
                }
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1){
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                    setPathState(6);
                }
                break;
            case 6:
            ///Initialize Transfer Intake Aiming
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5) {
                    IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                    IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
                    IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    setPathState(7);
                }
                break;
            case 7:
            ///Outtake Transfer Aiming
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {

                    OuttakeElbowRight.setPosition(OuttakeElbowPositionTransfer);
                    OuttakeWrist.setPosition(OuttakeWristPositionTransfer);
                    setPathState(8);
                }
                break;
            case 8:
            ///Transfer
                if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                    IntakeClaw.setPosition(IntakeClawPositionOpen);


                }
                if (pathTimer.getElapsedTimeSeconds() >= 0.35) {
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);

                    setPathState(9);
                }
                break;
            case 9:
            ///Path to Score first sample from floor
                if (pathTimer.getElapsedTimeSeconds()>=0.5) {
                    follower.followPath(sample1Score, true);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                    OuttakeWrist.setPosition(OuttakeWristPositionAiming);
                    setPathState(10);
                }
                break;
            case 10:
            ///Sliders Up to high basket and beginning aiming outtake claw
                if ((pathTimer.getElapsedTimeSeconds()>=0.15)) {
                    sliderMove(HIGH_BASKET);
                    OuttakeWrist.setPosition(OuttakeWristPositionAiming);
//                    OuttakeElbowRight.setPosition(OuttakeElbowPositionSpecimenScoring);

                    setPathState(11);
                }

                break;
            case 11:
            ///Aim to score
                if (pathTimer.getElapsedTimeSeconds() >= 2 && !follower.isBusy()) {
                    OuttakeWrist.setPosition(OuttakeWristPositionScoring);
                    setPathState(12);
                    }
                break;
            case 12:
            ///Score first sample from floor
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        OuttakeWrist.setPosition(OuttakeWristPositionInit);
                        IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                        IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                        IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                        setPathState(13);
                    }
                }
                break;
            case 13:
            ///Travel to second sample from floor
                if (pathTimer.getElapsedTimeSeconds() >= 0.15) {
                    follower.followPath(sample2, true);
                    sliderMove(0);

                    setPathState(15);
                }
                break;
            case 14:
                    if (pathTimer.getElapsedTimeSeconds() >= 0.15){

                        setPathState(16);
                    }
                    break;
            case 15:
                ///Grabs second sample from the floor
                    if (!follower.isBusy()){
                        IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
                        IntakeClaw.setPosition(IntakeClawPositionClose);
                        setPathState(16);
                    }
                    break;
                case 16:
                ///PRepare for transfer second sample
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.25) {
                        IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                        IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
                        IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                        setPathState(17);
                    }
                    break;
                case 17:
                ///Transfer Outtake Begins
                    if (pathTimer.getElapsedTimeSeconds() >= 0.35) {
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                        OuttakeElbowRight.setPosition(OuttakeElbowPositionTransfer);
                        OuttakeWrist.setPosition(OuttakeWristPositionTransfer);
                        setPathState(18);
                    }
                    break;
                case 18:
                    ///Transfer
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                        IntakeClaw.setPosition(IntakeClawPositionOpen);
                        setPathState(19);
                    }
                    break;
                case 19:
                    ///Path to Score second sample from floor
                    if (pathTimer.getElapsedTimeSeconds()>=0.5) {
                        follower.followPath(sample2Score, true);
                        OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                        OuttakeWrist.setPosition(OuttakeWristPositionAiming);
                        setPathState(20);
                    }
                    break;
                case 20:
                ///Resets
                    if ((pathTimer.getElapsedTimeSeconds()>=0.15)) {
                        sliderMove(HIGH_BASKET);
                        OuttakeWrist.setPosition(OuttakeWristPositionAiming);
                        setPathState(21);
                    }
                    break;
                case 21:
                ///Park
                    if (pathTimer.getElapsedTimeSeconds() >= 2 && !follower.isBusy()) {
                        OuttakeWrist.setPosition(OuttakeWristPositionScoring);
                        setPathState(22);
                    }
                    break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    follower.followPath(sample3);
                    if (pathTimer.getElapsedTimeSeconds() >= -.5) {
                        OuttakeWrist.setPosition(OuttakeWristPositionInit);
                        sliderMove(0);
                        IntakeSliderRight.setPosition(0.5);
                        IntakeSliderLeft.setPosition(0.5);
                        IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                        IntakeWrist.setPosition(0.75);
                        setPathState(23);
                    }
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() >= 2.5){
                    IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
            }
                if (pathTimer.getElapsedTimeSeconds() >= 3) {
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.15) {
                    IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                    IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
                    IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                    IntakeWrist.setPosition(IntakeWristInitialPosition);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds() >= 0.55) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionTransfer);
                    OuttakeWrist.setPosition(OuttakeWristPositionTransfer);
                    setPathState(26);
                }
                break;

            case 26:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    follower.followPath(sample3Score, true);
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    setPathState(27);
                }
                    break;
            case 27:
                if (pathTimer.getElapsedTimeSeconds()>=0.5) {
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                    OuttakeWrist.setPosition(OuttakeWristPositionAiming);
                    setPathState(28);
                }
                break;
            case 28:
                if ((pathTimer.getElapsedTimeSeconds()>=0.15)) {
                    sliderMove(HIGH_BASKET);
                    OuttakeWrist.setPosition(OuttakeWristPositionAiming);
                    setPathState(29);
                }
                break;
            case 29:
                if (pathTimer.getElapsedTimeSeconds() >= 2 && !follower.isBusy()) {
                    OuttakeWrist.setPosition(OuttakeWristPositionScoring);
                    setPathState(30);
                }
                break;
            case 30:
                if (pathTimer.getElapsedTimeSeconds() >= .25) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    setPathState(31);
                    }
                break;
            case 31:
                if (pathTimer.getElapsedTimeSeconds() >= 1){
                    OuttakeWrist.setPosition(OuttakeWristPositionInit);
                    OuttakeSliderLeft.setTargetPosition(0);
                    OuttakeSliderLeft.setPower(1);
                    OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1) {

                    //follower.followPath(park);
                    setPathState(-1);
                }

        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Range",range);
        telemetry.addData("OuttakeRight",OuttakeElbowRight.getPosition());
        telemetry.addData("Path Time",pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        //drive.setDrivePowers(pv);

        //Outtake
        // Sliders Mapping and Setup


        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        OuttakeSliderLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        Crush.getInstance().setInitialPositions(initialPositionLeft);

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeClaw.setPosition(OuttakeClawPositionClose);

        OuttakeElbowRight.setDirection(Servo.Direction.REVERSE);
        OuttakeWrist.setDirection(Servo.Direction.REVERSE);

        //Intake
        //Servo Sliders Mapping and Setup



        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");

        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");


        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);
        IntakeSliderLeft.scaleRange(0.0, 1.0);
        IntakeSliderRight.scaleRange(0.0, 1.0);

        OuttakeWrist.setPosition(OuttakeWristPositionInit);
        OuttakeElbowRight.setPosition(0.5);

        IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
        IntakeSliderRight.setPosition(IntakeSliderPositionIN);
        IntakeElbowRight.setPosition(.3);
        IntakeClaw.setPosition(IntakeClawPositionOpen);
        IntakeWrist.setPosition(0.5);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);



    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {

    }

    public PathChain getSample1() {
        return sample1;
    }

    public PathChain getSample2() {
        return sample2;
    }

    public PathChain getRecoger() {
        return sample3;
    }
    private void intake(int pos){
        IntakeSliderRight.setPosition(pos);
        IntakeSliderLeft.setPosition(pos);
        IntakeElbowRight.setPosition(pos);
    }
}

