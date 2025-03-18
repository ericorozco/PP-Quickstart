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
@Autonomous(name = "Test2", group = "Auto")
public class nms2 extends OpMode {

    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo IntakeElbowRight;
    private Servo IntakeElbowLeft;
    private Servo IntakeWrist;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    private Servo IntakeClaw;
    final int HIGH_BASKET = 3600;
    final int HIGH_CHAMBER = 700;
    public int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionOut = 0.17;
    final double OuttakeElbowPositionSpecimenScoring = 0.30;
    final double OuttakeElbowPositionIn = 0.85;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;
    final double IntakeElbowPositionIn = 0.85;
    final double IntakeElbowPositionOut = 0.23;
    final double IntakeElbowPositionGrab = 0.2;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;

    private enum IntakeState{
        IN,
        OUT
    }
    final double IntakeSliderPositionOUT = 0.57;
    final double IntakeSliderPositionIN = 0.45;
    final double IntakeSliderPositionTransfer = 0.21;
    final double IntakeClawPositionClose = 0.60;
    final double IntakeClawPositionOpen = 0.35;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int initialPos;
    public static double IntakeWristRight = 0.2;

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
    public static double scorePoseX = 19.0;
    public static double scorePoseY = 125.0;
    private final Pose highBasketPose = new Pose(scorePoseX, scorePoseY, Math.toRadians(315));
    public static double hbControlX = 27.0;
    public static double hbControlY = 118.0;
    private final Pose controlHighBasketPose = new Pose (hbControlX,hbControlY, Math.toRadians(0));
    /** Lowest (First) Sample from the Spike Mark */
    public static double samplePos1X = 26.25;
    public static double samplepos1Y = 122;
    private final Pose samplepos1 = new Pose(samplePos1X, samplepos1Y, Math.toRadians(0));
    private final Pose controlsamplepos1 = new Pose(80, 36, Math.toRadians(0));
    public static double samplePos3X = 44.7;
    public static double samplepos3Y = 123.2;
    private final Pose samplepos3 = new Pose(samplePos3X, samplepos3Y, Math.toRadians(90));
    private final Pose samplepos2 = new Pose(samplePos2X, samplepos2Y, Math.toRadians(0));
    public static double samplePos2X = 26.25;
    public static double samplepos2Y = 130.2;
    private final Pose controlsamplepos2 = new Pose(75, 26, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose samplepush2 = new Pose(samplepush2X, samplepush2Y, Math.toRadians(0));
    public static double samplepush2X = 19.0;
    public static double samplepush2Y = 17.0;

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
        OuttakeSliderRight.setTargetPosition(initialPositionRight + Position);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setPower(.68);
        OuttakeSliderRight.setPower(.7);
        /*while (OuttakeSliderLeft.isBusy() && OuttakeSliderRight.isBusy()){

        }
        OuttakeSliderLeft.setPower(0);
        OuttakeSliderRight.setPower(0);*/
    }


    public void outtakeElbow (double SPosition){
        OuttakeElbowLeft.setPosition(SPosition);
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
                .addPath(new BezierLine(new Point(samplepos1),new Point(highBasketPose)))
                .setLinearHeadingInterpolation(samplepos1.getHeading(), highBasketPose.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        sample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose),new Point(samplepos2)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), samplepos2.getHeading())
                .build();
        sample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose),new Point(samplepos3)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), samplepos3.getHeading())
                .build();
        sample3Score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepos3),new Point(highBasketPose)))
                .setLinearHeadingInterpolation(samplepos3.getHeading(), highBasketPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(highBasketPose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(highBasketPose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
            ///Aims High Basket
                follower.followPath(scorePreload, true);
                sliderMove(HIGH_BASKET + 1);
                IntakeClaw.setPosition(OuttakeClawPositionOpen);
                setPathState(1);
                break;

            case 1:
            ///Prepares Outtake Elbows for scoring
                if (follower.getPose().getX() >= startPose.getX() && follower.getPose().getY() >= startPose.getY()) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionMiddle);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                    setPathState(2);
                }
                break;
            case 2:
            ///Aims Preload with Elbow
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    setPathState(3);
                }
                break;
            case 3:
            ///Scores Preload
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    setPathState(4);
                }
                break;
            case 4:
            ///Moves towards 1st Sample
                if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                    //follower.followPath(sample2push);
                    outtakeElbow(OuttakeElbowPositionMiddle);
                    sliderMove(0);
                    follower.followPath(sample1);
                    IntakeSliderLeft.setPosition(IntakeSliderPositionOUT);
                    IntakeSliderRight.setPosition(IntakeSliderPositionOUT);
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    setPathState(6);
                }
                break;
            case 5:
            ///Pushes 1st Sample
                if (!follower.isBusy()) {
                    IntakeSliderLeft.setPosition(IntakeSliderPositionOUT);
                    IntakeSliderRight.setPosition(IntakeSliderPositionOUT);
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                    IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                    setPathState(6);
                }
                break;
            case 6:
            ///Moves Towards 2nd Sample
                if (!follower.isBusy()) {
                    IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                    IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                    setPathState(7);
                }
                break;
            case 7:
            ///Pushes 2nd Sample
                if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                    IntakeElbowLeft.setPosition(IntakeElbowPositionGrab);
                    IntakeElbowRight.setPosition(IntakeElbowPositionGrab);

                    setPathState(8);
                }
                break;
            case 8:
            ///Prepares to grab 1st Specimen
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                    IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                    IntakeSliderLeft.setPosition(IntakeSliderPositionTransfer);
                    IntakeSliderRight.setPosition(IntakeSliderPositionTransfer);
                    setPathState(9);
                }
                break;
            case 9:
            ///Lines up to 1st Specimen
                if (pathTimer.getElapsedTimeSeconds()>=0.5) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionIn);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionIn);
                    setPathState(10);
                }
                break;
            case 10:
            ///Grabs 1st Specimen
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                        setPathState(11);
                    }

                break;
            case 11:
            ///Picks up Claw and Sliders
                if (pathTimer.getElapsedTimeSeconds() >= .5) {
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    follower.followPath(sample1Score, true);
                    sliderMove(HIGH_BASKET);
                    setPathState(12);
                    }
                break;
            case 12:
            ///Goes to Score 1st Specimen
                if (!follower.isBusy()) {
                    //follower.followPath(score, true);
                    setPathState(13);
                }
                break;
            case 13:
            ///Scores 1st Specimen
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    setPathState(14);
                    pathTimer.resetTimer();
                }
                break;
                case 14:
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5){
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    }
                    if (follower.getPose().getX() >= scorePoseX - 0.5 && follower.getPose().getY() >= scorePoseY - 0.5 || pathTimer.getElapsedTimeSeconds() >= 1) {
                        outtakeElbow(OuttakeElbowPositionMiddle);
                        sliderMove(0);
                        setPathState(15);
                    }
                    break;
                case 15:
                    if (!follower.isBusy()){
                        //follower.followPath(recogerFromScore,true);
                        follower.followPath(sample2);
                        setPathState(16);
                    }
                    break;
                case 16:
                ///Lines up for 2rd Specimen
                    if (!follower.isBusy()) {
                        IntakeSliderLeft.setPosition(IntakeSliderPositionOUT);
                        IntakeSliderRight.setPosition(IntakeSliderPositionOUT);
                        IntakeClaw.setPosition(IntakeClawPositionOpen);
                        IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                        IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                        setPathState(17);
                        }
                    break;
                case 17:
                ///Grabs 2rd Specimen
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        IntakeElbowLeft.setPosition(IntakeElbowPositionGrab);
                        IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
                        setPathState(18);
                    }
                    break;
                case 18:
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        IntakeClaw.setPosition(IntakeClawPositionClose);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 1) {
                        IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                        IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                        IntakeSliderLeft.setPosition(IntakeSliderPositionTransfer);
                        IntakeSliderRight.setPosition(IntakeSliderPositionTransfer);
                        setPathState(19);
                    }
                    break;
                case 19:
                ///Scores 2nd Specimen
                    if (pathTimer.getElapsedTimeSeconds()>=0.5) {
                        OuttakeElbowLeft.setPosition(OuttakeElbowPositionIn);
                        OuttakeElbowRight.setPosition(OuttakeElbowPositionIn);
                        setPathState(20);
                    }
                    break;
                case 20:
                ///Resets
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                        setPathState(21);
                    }
                    break;
                case 21:
                ///Park
                    if (pathTimer.getElapsedTimeSeconds() >= .5) {
                        IntakeClaw.setPosition(IntakeClawPositionOpen);
                        follower.followPath(sample1Score, true);
                        sliderMove(HIGH_BASKET);
                        setPathState(22);
                    }
                    break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5){
                OuttakeClaw.setPosition(OuttakeClawPositionOpen);
            }
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    outtakeElbow(OuttakeElbowPositionMiddle);
                    sliderMove(0);
                    IntakeWrist.setPosition(IntakeWristRight);
                    follower.followPath(sample3);
                    setPathState(25);
                }
                break;
            case 24:
                if(!follower.isBusy()){
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()){
                    IntakeSliderLeft.setPosition(IntakeSliderPositionOUT);
                    IntakeSliderRight.setPosition(IntakeSliderPositionOUT);
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                    IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                    setPathState(26);
                }
                break;

            case 26:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    IntakeElbowLeft.setPosition(IntakeElbowPositionGrab);
                    IntakeElbowRight.setPosition(IntakeElbowPositionGrab);
                    IntakeClaw.setPosition(IntakeClawPositionClose);
                }

                    if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                        IntakeWrist.setPosition(0.5);
                        IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                        IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                        IntakeSliderLeft.setPosition(IntakeSliderPositionTransfer);
                        IntakeSliderRight.setPosition(IntakeSliderPositionTransfer);
                        setPathState(27);
                }
                    break;
            case 27:
                if (pathTimer.getElapsedTimeSeconds()>=1) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionIn);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionIn);
                    setPathState(28);
                }
                break;
            case 28:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    setPathState(29);
                }
                break;
            case 29:
                if (pathTimer.getElapsedTimeSeconds() >= .5) {
                    IntakeClaw.setPosition(IntakeClawPositionOpen);
                    follower.followPath(sample3Score, true);
                    sliderMove(HIGH_BASKET);
                    setPathState(30);
                }
                break;
            case 30:
                if (pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    setPathState(31);
                }
                break;
            case 31:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5){
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    outtakeElbow(OuttakeElbowPositionMiddle);
                    sliderMove(0);
                    IntakeWrist.setPosition(IntakeWristRight);
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
        telemetry.addData("OuttakeLeft",OuttakeElbowLeft.getPosition());
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


        OuttakeSliderRight = hardwareMap.get(DcMotorEx.class, "OuttakeSliderRight");
        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        OuttakeSliderRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        OuttakeSliderLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        OuttakeSliderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        initialPositionRight = OuttakeSliderRight.getCurrentPosition();
        Crush.getInstance().setInitialPositions(initialPositionRight, initialPositionLeft);

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");
        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowRight.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        //Intake
        //Servo Sliders Mapping and Setup



        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");

        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
        IntakeElbowLeft = hardwareMap.get(Servo.class, "IntakeElbowLeft");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");


        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);
        IntakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);
        IntakeSliderLeft.scaleRange(0.0, 1.0);
        IntakeSliderRight.scaleRange(0.0, 1.0);

        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        IntakeSliderLeft.setPosition(0);
        IntakeSliderRight.setPosition(0);
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
}

