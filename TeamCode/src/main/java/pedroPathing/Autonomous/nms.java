package pedroPathing.Autonomous;
import static android.os.SystemClock.elapsedRealtime;
import static android.os.SystemClock.sleep;

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

@Autonomous(name = "Test", group = "Auto")
public class nms extends OpMode {

    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    final int HIGH_BASKET = 3600;
    final int HIGH_CHAMBER = 700;
    public int initialPositionLeft, initialPositionRight;

    final double OuttakeElbowPositionOut = 0.21;
    final double OuttakeElbowPositionSpecimenScoring = 0.30;
    final double OuttakeElbowPositionIn = 0.72;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;

    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;

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
    private final Pose startPose = new Pose(9, 62, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(32, 62, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose samplepos1 = new Pose(60, 24, Math.toRadians(0));
    private final Pose controlsamplepos1 = new Pose(5, 18, Math.toRadians(0));
    private final Pose controlsamplepos12 = new Pose(85, 48, Math.toRadians(0));
    private final Pose samplepush1 = new Pose(22, 24, Math.toRadians(0));
    private final Pose samplepos2 = new Pose(63, 13, Math.toRadians(0));
    private final Pose controlsamplepos2 = new Pose(75, 26, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose samplepush2 = new Pose(25, 13, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup = new Pose(35, 35, Math.toRadians(0));
    private final Pose pickup2 = new Pose(24, 35, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(10, 20, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(11, 11, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain sample1, samplepushUno, sample2, samplepushDos, recoger, recogerFromScore, score;
    private boolean range;
    public void sliderMove (int Position){
        OuttakeSliderLeft.setTargetPosition(initialPositionLeft + Position);
        OuttakeSliderRight.setTargetPosition(initialPositionRight + Position);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setPower(.4);
        OuttakeSliderRight.setPower(.4);
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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());



        /* Gets in Position to push the 1st Sample into the Observation Zone */
        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),new Point(controlsamplepos1),new Point(controlsamplepos12), new Point(samplepos1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), samplepos1.getHeading())
                .build();
        samplepushUno = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepos1),new Point(samplepush1)))
                .setConstantHeadingInterpolation(samplepush1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        sample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(samplepush1),new Point(controlsamplepos2), new Point(samplepos2)))
                .setConstantHeadingInterpolation(samplepos2.getHeading())
                .addPath(new BezierLine(new Point(samplepos2),new Point(samplepush2)))
                .setConstantHeadingInterpolation(samplepush2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        recoger = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplepush2), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .addPath(new BezierLine(new Point(pickup), new Point(pickup2)))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();
        recogerFromScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .addPath(new BezierLine(new Point(pickup), new Point(pickup2)))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
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
                if (follower.getPose().getX() >= 9 && follower.getPose().getY() >= 62) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionMiddle);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionMiddle);
                }

                follower.followPath(scorePreload,true);
                sliderMove(HIGH_CHAMBER);

                if (follower.getPose().getX() >= 28 && follower.getPose().getY() >= 62) {
                    OuttakeElbowLeft.setPosition(OuttakeElbowPositionSpecimenScoring);
                    OuttakeElbowRight.setPosition(OuttakeElbowPositionSpecimenScoring);
                    setPathState(1);
                }



                break;
            case 1:
                if (follower.getPose().getX() >= 31 && follower.getPose().getY() >= 62 && pathTimer.getElapsedTimeSeconds() >= 4){
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    setPathState(2);
                }

                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                follower.followPath(sample1,true);
                if (follower.getPose().getX() >= 60 && follower.getPose().getY() >= 24){
                    outtakeElbow(OuttakeElbowPositionMiddle);
                    sliderMove(initialPos);
                    follower.followPath(samplepushUno);
                    setPathState(3);
                }

                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() >= 22 && follower.getPose().getY() >= 24){
                    setPathState(-1);
                }
                break;
            case 4:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score,true);
                    sliderMove(HIGH_CHAMBER);
                    setPathState(5);
                }
                break;
            case 5:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(recogerFromScore,true);
                    sleep(1000);
                    setPathState(6);
                }
                break;
            case 6:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score,true);
                    sliderMove(HIGH_CHAMBER);
                    setPathState(7);
                }
                break;
            case 7:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    sliderMove(initialPositionLeft);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;

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

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        OuttakeClaw.setPosition(OuttakeClawPositionClose);
        IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
        IntakeSliderRight.setPosition(IntakeSliderPositionIN);

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
}

