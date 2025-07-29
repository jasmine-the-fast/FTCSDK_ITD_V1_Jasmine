package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;
import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Sample_Auto", group = "OpMode")
public class Samples extends OpMode {
    BTRobotV1 robot = new BTRobotV1(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState, commandState;

    public String Color_Alliance = null;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(13, 128, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(20, 125, Math.toRadians(343));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(20, 132, Math.toRadians(343));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(20, 132, Math.toRadians(10));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));
    /**Intake from Submersible */
    private final Pose pickupSubPose = new Pose(65, 95, Math.toRadians(270));

    /** Used as the control point for the BeizerCurve for SubPose*/
    private final Pose pickupSubControlPose = new Pose(55, 130, Math.toRadians(270));

    /** Park Pose for our robot, after we do all of the scoring. */

    /** Used as the control point for the BeizerCurve for parkPose*/
    private final Pose parkControlPose = new Pose(63, 110, Math.toRadians(90));
    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, grabSubPose1, scoreSubPose1, grabSubPose2, scoreSubPose2;

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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        grabSubPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupSubControlPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();

        scoreSubPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(pickupSubControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading())
                .build();

        grabSubPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupSubControlPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();

        scoreSubPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(pickupSubControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void timeWristOut(){
        actionTimer.resetTimer();
        robot.Setup_Deposit_Arm(0.5);
        robot.Setup_Deposit_Wrist(0.1);
        while (actionTimer.getElapsedTimeSeconds()<5){
            telemetry.addLine("wait for wrist out");
        }
    }
    public void timeArmForTransfer(){
        actionTimer.resetTimer();
        robot.Setup_Deposit_Arm(0.18);
        while (actionTimer.getElapsedTimeSeconds()<3){
            telemetry.addLine("wait for wrist out");
        }
    }
    public void timeWristIn() {
        actionTimer.resetTimer();
        robot.Setup_Deposit_Arm(0.13);
        robot.Setup_Deposit_Wrist(0.28);
        while (actionTimer.getElapsedTimeSeconds() < 5) {
            telemetry.addLine("wait for wrist in");
        }
    }
    public void autonomousPathUpdate() {
        telemetry.addData("autonomousPathUpdate - path state", pathState);
        switch (pathState) {
            case 0:
                robot.Setup_Intake_Pose_RTP(true);
                robot.Setup_Horizontal_Lift(0.0);
                follower.followPath(scorePreload,true);
                sleepMethod(0.5);
                robot.Setup_Deposit_Arm(0.55);
                sleepMethod(1.5);
                robot.HighBasketScore();
                sleepMethod(0.1);
                setPathState(1);
                telemetry.addData("autonomousPathUpdate - 1 path state", pathState);
                break;
            case 1:
                if(!follower.isBusy()) {
                    robot.Setup_Deposit_Claw(true);
                    robot.Setup_Deposit_Arm(0.15);
                    sleepMethod(1.5);
                    robot.TransferSample();
                    sleepMethod(1.5);
                    follower.followPath(grabPickup1,true);
                    robot.Setup_Deposit_Arm(0.15);
                    robot.Setup_Intake_Pose_RTP(false);
                    sleepMethod(1.0);
                    if(follower.getPose().getX()>=17){
                        robot.Intake(-1.0);
                        robot.Setup_Horizontal_Lift(1.0);
                        sleepMethod(0.3);
                    }
                    sleepMethod(0.3);
                    if(robot.HLL.getPosition() >= 0.9 || robot.HLR.getPosition() >= 0.9){
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    robot.Setup_Intake_Pose_RTP(true);
                    robot.Setup_Horizontal_Lift(0.0);
                    robot.TransferSample();
                    follower.followPath(scorePickup1,true);
                    sleepMethod(0.5);
                    robot.Setup_Deposit_Arm(0.55);
                    sleepMethod(1.5);
                    robot.HighBasketScore();
                    sleepMethod(0.1);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    robot.Setup_Deposit_Claw(true);
                    robot.Setup_Deposit_Arm(0.15);
                    sleepMethod(1.5);
                    robot.TransferSample();
                    sleepMethod(1.5);
                    follower.followPath(grabPickup2,true);
                    robot.Setup_Deposit_Arm(0.15);
                    robot.Setup_Intake_Pose_RTP(false);
                    sleepMethod(1.0);
                    if(follower.getPose().getX()>=17){
                        robot.TransferSample();
                        robot.Intake(-1.0);
                        robot.Setup_Horizontal_Lift(1.0);
                        sleepMethod(0.3);
                    }
                    robot.TransferSample();
                    sleepMethod(0.3);
                    if(robot.HLL.getPosition() >= 0.9 || robot.HLR.getPosition() >= 0.9){
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    robot.Setup_Intake_Pose_RTP(true);
                    robot.Setup_Horizontal_Lift(0.0);
                    follower.followPath(scorePickup2,true);
                    sleepMethod(0.5);
                    robot.Setup_Deposit_Arm(0.55);
                    sleepMethod(1.5);
                    robot.HighBasketScore();
                    sleepMethod(0.1);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    robot.Setup_Deposit_Claw(true);
                    robot.Setup_Deposit_Arm(0.15);
                    sleepMethod(1.5);
                    robot.TransferSample();
                    sleepMethod(1.5);
                    follower.followPath(grabPickup3,true);
                    robot.Setup_Deposit_Arm(0.15);
                    robot.Setup_Intake_Pose_RTP(false);
                    sleepMethod(1.0);
                    if(follower.getPose().getX()>=17){
                        robot.TransferSample();
                        robot.Intake(-1.0);
                        robot.Setup_Horizontal_Lift(1.0);
                        sleepMethod(0.3);
                    }
                    robot.TransferSample();
                    sleepMethod(0.3);
                    if(robot.HLL.getPosition() >= 0.9 || robot.HLR.getPosition() >= 0.9){
                        setPathState(4);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    robot.Setup_Intake_Pose_RTP(true);
                    robot.Setup_Horizontal_Lift(0.0);
                    follower.followPath(scorePickup3,true);
                    sleepMethod(0.5);
                    robot.Setup_Deposit_Arm(0.55);
                    sleepMethod(1.0);
                    robot.HighBasketScore();
                    sleepMethod(0.1);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(grabSubPose1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSubPose1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(grabSubPose2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSubPose2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(park,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
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
        robot.Intake_Poop(robot.intakeColor.equals(Color_Alliance) || robot.intakeColor.equals("Yellow"));
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.initialize(true);
        robot.colorSensor.enableLed(true);
        robot.getColor();
        Color_Alliance = "Blue";
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        PoseStorage.CurrentPose = follower.getPose();
    }

    public void sleepMethod(double timeInSeconds) {
        long temp = (long) (timeInSeconds * 1000);
        try {
            //Pause the current thread for x
            Thread.sleep(temp);
        } catch (InterruptedException e) {
            // Handle the InterruptedException, which occurs if another thread interrupts this one
            System.err.println("Thread interrupted during sleep: " + e.getMessage());
            // Re-interrupt the current thread to indicate that it was interrupted
            Thread.currentThread().interrupt();
        }
    }
}
