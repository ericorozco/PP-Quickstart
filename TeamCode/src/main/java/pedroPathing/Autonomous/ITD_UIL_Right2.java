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
@Autonomous(name = "ITD-UIL-Right2", group = "Auto")
public class ITD_UIL_Right2 extends OpMode {

    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeWrist;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 600;
    final int HIGH_CHAMBER = 2200;
    public int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionOut = 0.25;
    final double OuttakeElbowPositionSpecimenScoring = 0.25;
    final double OuttakeElbowPositionSpecimenAiming = 0.75;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeClawPositionClose = 0.35;
    final double OuttakeClawPositionOpen = 0.70;

    final double OuttakeWristPositionScoring = 0.75;
    final double OuttakeWristPositionScorePreload = 1.0;
    final double OuttakeWristPositionOut = 0.5;
    final double OuttakeWristPositionAfterScore = 0.6;

    final double OuttakeWristPositionGrab = 1.0;
    final double OuttakeWristPositionSpecimenAiming = 0.45;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
    private Servo IntakeWrist;
    final double IntakeWristInitialPosition = 0.5;
    private Servo IntakeElbowRight;
    private Servo block;


    public PathChain getRecoger1() {
        return recogerFromScore;
    }

    private enum IntakeState{
        IN,
        OUT
    }
    final double IntakeSliderPositionOUT = 0.60;
    final double IntakeSliderPositionIN = 0.0;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int initialPos;

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
    private final Pose startPose = new Pose(9, 62, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    public static double scorePoseX = 39.0;
    public static double scorePoseY = 65.0;
    private final Pose scorePose = new Pose(scorePoseX, scorePoseY, Math.toRadians(0));
    private final Pose scorePoseLast = new Pose(scorePoseX+10, scorePoseY+10, Math.toRadians(0));
    public static double scoreToSampleX = 58.0;
    public static double scoreToSampleY = 24.0;
    private final Pose scoreToSample = new Pose(scoreToSampleX,scoreToSampleY,Math.toRadians(0));
    public static double Control1x = 16.0;
    public static double Control1y = 17.0;
    private final Pose score2sampleControl1 = new Pose(Control1x,Control1y,Math.toRadians(0));
    public static double backX = 28.0;
    public static double backY = 62.0;
    private final Pose back = new Pose(backX,backY,Math.toRadians(0));
    private final Pose Control1 = new Pose(10,36,Math.toRadians(0));

    public static double Control2x = 76.0;
    public static double Control2y = 46.0;
    private final Pose score2sampleControl2 = new Pose(Control2x,Control2y,Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    public static double samplePos1X = 58.0;
    public static double samplepos1Y = 25.0;
    private final Pose samplepos1 = new Pose(samplePos1X, samplepos1Y, Math.toRadians(0));
    private final Pose Sample1to2Control1 = new Pose(74,33.4,Math.toRadians(0));
    private final Pose controlsamplepos1 = new Pose(80, 36, Math.toRadians(0));
    public static double samplePos12X = 57.0;
    public static double samplepos12Y = 31.0;
    private final Pose controlsamplepos12 = new Pose(samplePos12X, samplepos12Y, Math.toRadians(0));
    private final Pose samplepush1 = new Pose(samplepush1X, samplepush1Y, Math.toRadians(0));
    public static double samplepush1X = 16.0;
    public static double samplepush1Y = 24.0;
    private final Pose samplepos2 = new Pose(samplePos2X, samplepos2Y, Math.toRadians(0));
    public static double samplePos2X = 52.0;
    public static double samplepos2Y = 13.0;
    private final Pose controlsamplepos2 = new Pose(20, 51, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    public static double samplepush2X = 25.0;
    public static double samplepush2Y = 13.0;
    private final Pose samplepush2 = new Pose(samplepush2X, samplepush2Y, Math.toRadians(0));

    public static double samplePos3X = 50;
    public static double samplePos3Y = 25;
    public static double samplePosControl3X = 23;
    public static double samplePosControl3Y = 29;

    private final Pose push2control1 = new Pose(84.5,32,0);
    private final Pose push2control2 = new Pose(73.3,8.9,0);
    private final Pose push2control3 = new Pose(49.9,12.6,0);
    private final Pose push2control4 = new Pose(31,13.7,0);

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup = new Pose(14, 13, Math.toRadians(0));
//    private final Pose pickup2 = new Pose(14, 13, Math.toRadians(0));
    private final Pose pickup2 = new Pose(14, 35, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(15, 20, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(11, 11, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain sample1, sample3, sample2push,sample2push2, samplepushUno, sample2, samplepushDos, recoger, prono, recogerFromScore, score, scoreLast;
    private boolean range;
    public void sliderMove (int Position){
        OuttakeSliderLeft.setTargetPosition(initialPositionLeft + Position);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setPower(.60);
        /*while (OuttakeSliderLeft.isBusy() && OuttakeSliderRight.isBusy()){

        }
        OuttakeSliderLeft.setPower(0);
        OuttakeSliderRight.setPower(0);*/
    }


    public void outtakeElbow (double SPosition){
        OuttakeWrist.setPosition(SPosition);
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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());



        /* Gets in Position to push the 1st Sample into the Observation Zone */
        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),new Point(score2sampleControl1),new Point(score2sampleControl2), new Point(samplepos1)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        sample2push = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(back)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierCurve(new Point(back),new Point(Control1), new Point(samplepos1)))
                .setConstantHeadingInterpolation(back.getHeading())
                .addPath(new BezierLine(new Point(samplepos1),new Point(scoreToSample)))
                .setConstantHeadingInterpolation(scoreToSample.getHeading())
                .build();

        samplepushUno = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepos1),new Point(samplepush1)))
                .setConstantHeadingInterpolation(samplepush1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        sample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepush1), new Point(controlsamplepos12)))
                .setConstantHeadingInterpolation(controlsamplepos12.getHeading())
                .addPath(new BezierLine(new Point(controlsamplepos12),new Point(samplepos2)))
                .setConstantHeadingInterpolation(samplepos2.getHeading())
                .build();

        samplepushDos = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(samplepush1),new Point(push2control1),new Point(samplepos2)))
                .setConstantHeadingInterpolation(samplepush2.getHeading())
                .addPath(new BezierLine(new Point(samplepos2),new Point(samplepush2)))
                .setConstantHeadingInterpolation(samplepush2.getHeading())
                .build();

        sample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(samplepush2), new Point(samplepos1)))
                .setConstantHeadingInterpolation(controlsamplepos12.getHeading())
                .addPath(new BezierCurve(new Point(controlsamplepos12),new Point(samplepos2)))
                .setConstantHeadingInterpolation(samplepos2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        recoger = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepush2), new Point(pickup)))
                .setConstantHeadingInterpolation(samplepush2.getHeading())
                .build();
        prono = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup), new Point(pickup2)))
                .setConstantHeadingInterpolation(samplepush2.getHeading())
                .build();
        recogerFromScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        scoreLast = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(scorePoseLast)))
                .setConstantHeadingInterpolation(scorePoseLast.getHeading())
                .build();



        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
            ///Goes to Submersible
                follower.setMaxPower(0.9);

                follower.followPath(scorePreload, true);
                sliderMove(HIGH_CHAMBER);
                outtakeElbow(OuttakeElbowPositionMiddle);
                OuttakeElbowRight.setPosition(OuttakeElbowPositionSpecimenAiming);
                OuttakeWrist.setPosition(OuttakeWristPositionSpecimenAiming);
                setPathState(2);
                break;
            case 1:
            ///Prepares Outtake Elbows for scoring
                if (follower.getPose().getX() >= scorePoseX && follower.getPose().getY() >= scorePoseY) {
                    setPathState(2);
                }
                break;
            case 2:
            ///Scores Preload
                if (follower.getPose().getX() >= scorePoseX - 1.25 && follower.getPose().getY() >= scorePoseY - 0.65) {

                    OuttakeWrist.setPosition(OuttakeWristPositionScorePreload);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionSpecimenScoring);
                    setPathState(3);
                }
                break;
            case 3:
            ///Resets everything to Init. Pos.
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    sliderMove(0);
                    follower.followPath(sample1);
                    setPathState(5);
                }
                break;
            case 4:
            ///Moves towards 1st Sample
                if(pathTimer.getElapsedTimeSeconds()>0.15 && pathTimer.getElapsedTimeSeconds()<0.75) {
                    sliderMove(0);
                    OuttakeWrist.setPosition(OuttakeWristPositionAfterScore);
                }
                if (pathTimer.getElapsedTimeSeconds() >= 0.65) {
                    follower.followPath(sample2push);
                    setPathState(5);
                }
                break;
            case 5:
            ///Pushes 1st Sample
                if (!follower.isBusy()) {
                    follower.followPath(samplepushUno);
                    OuttakeElbowRight.setPosition(0.25);
                    OuttakeWrist.setPosition(0.5);
                    setPathState(7);
                }
                break;
            case 6:
            ///Moves Towards 2nd Sample
                if (!follower.isBusy()) {
                    follower.followPath(samplepushDos);

                    setPathState(7);
                }
                break;
            case 7:
            ///Pushes 2nd Sample
                if (!follower.isBusy()) {
                    follower.followPath(samplepushDos);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    OuttakeWrist.setPosition(OuttakeWristPositionOut);
                    setPathState(8);
                }
                break;
            case 8:
            ///Prepares to grab 1st Specimen
                if (!follower.isBusy()) {
                    follower.followPath(recoger);
                    setPathState(10);
                }
                break;
            case 9:
            ///Lines up to 1st Specimen
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    follower.followPath(prono);
                    setPathState(10);
                }
                break;
            case 10:
            ///Grabs 1st Specimen
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 1.25) {
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                        setPathState(11);
                    }
                }
                break;
            case 11:

            ///Picks up Claw and Sliders
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        //outtakeElbow(OuttakeElbowPositionMiddle);
                        OuttakeElbowRight.setPosition(0.75);
                        OuttakeWrist.setPosition(0.45);
                        setPathState(12);
                    }
                }
                break;
            case 12:
            ///Goes to Score 1st Specimen
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(score, true);

                    setPathState(-1);
                }
                break;
            case 13:
            ///Scores 1st Specimen
                if(pathTimer.getElapsedTimeSeconds()>0.5 && pathTimer.getElapsedTimeSeconds()<1){
                    sliderMove(HIGH_CHAMBER);
                }
                if (follower.getPose().getX() >= scorePoseX - 1.5 && follower.getPose().getY() >= scorePoseY - 0.6) {
                    OuttakeWrist.setPosition(0.75);
                    OuttakeElbowRight.setPosition(0.25);
                    setPathState(14);
                }
                break;
                case 14:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                        //outtakeElbow(OuttakeElbowPositionMiddle);
                        sliderMove(0);
                        setPathState(15);
                        pathTimer.resetTimer();
                    }
                    break;
                case 15:
                    if (!follower.isBusy()){
                        follower.followPath(recogerFromScore,true);
                        OuttakeElbowRight.setPosition(0.25);
                        OuttakeWrist.setPosition(0.5);
                        setPathState(16);
                    }
                    break;
                case 16:
                ///Lines up for 2rd Specimen
                    if (!follower.isBusy()) {
                        follower.setMaxPower(0.5);
                        follower.followPath(prono);
                        setPathState(17);
                        pathTimer.resetTimer();
                        }
                    break;
                case 17:
                ///Grabs 2rd Specimen
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                            OuttakeClaw.setPosition(OuttakeClawPositionClose);
                            setPathState(18);
                        }
                    }
                    break;
                case 18:
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() >= 0.45) {
                            setPathState(181);
                            follower.setMaxPower(0.9);
                            follower.followPath(score);
//                            sliderMove(HIGH_CHAMBER+50);
                            OuttakeElbowRight.setPosition(0.75);
                            OuttakeWrist.setPosition(0.45);
                        }
                    }
                    break;
            case 181:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    setPathState(19);
                    sliderMove(HIGH_CHAMBER+50);

                }
                break;
                case 19:
                ///Scores 2nd Specimen
//                    if (follower.getPose().getX() >= scorePoseX - 0.6 && follower.getPose().getY() >= scorePoseY - 0.6) {
                    if (!follower.isBusy()) {
                        OuttakeWrist.setPosition(0.75);
                        OuttakeElbowRight.setPosition(0.25);
                        setPathState(20);
                        pathTimer.resetTimer();
                    }
                    break;
                case 20:
                ///Resets
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1) {
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                        OuttakeElbowRight.setPosition(0.75);
                        OuttakeWrist.setPosition(0.45);
                        sliderMove(0);
                        setPathState(21);
                    }
                    break;
                case 21:
                ///Park
                    if (!follower.isBusy()){
                        follower.followPath(park);
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
        telemetry.addData("OuttakeLeft",OuttakeWrist.getPosition());
        telemetry.addData("InitPos",initialPositionLeft);
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
        //Crush.getInstance().setInitialPositions(initialPositionRight, initialPositionLeft);

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        block = hardwareMap.get(Servo.class, "block");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        OuttakeWrist.setDirection(Servo.Direction.REVERSE);

        //Intake
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");
        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
        IntakeSliderRight.setPosition(IntakeSliderPositionIN);
        OuttakeWrist.setPosition(0.25);
        OuttakeElbowRight.setPosition(0.5);
        IntakeElbowRight.setPosition(0.30);
        IntakeWrist.setPosition(IntakeWristInitialPosition);
        block.setPosition(0.5);


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
        return recoger;
    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        //OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeWrist.setPosition(OuttakeElbowTargetPosition);
    }
    private void IntakeSliderMove(double position){
        IntakeSliderLeft.setPosition(IntakeSliderLeft.getPosition() + position);
        IntakeSliderRight.setPosition(IntakeSliderRight.getPosition() + position);

    }
}

