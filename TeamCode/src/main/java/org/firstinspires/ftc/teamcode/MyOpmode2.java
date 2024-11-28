package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Config
@Autonomous(group = "drive")
public class MyOpmode2 extends LinearOpMode {
    private Robot robot;
    private ScheduledFuture<?> lastArmRaise, lastSliderRaise;
    public static double X_BackDrop = -26,Y_BackDrop = 34;  ///pt start langa Backdrop
    @Override
    public void runOpMode() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())//bodo
//                .back(20)
//                .turn(Math.toRadians(45))
//                .forward(10)
//                .turn(Math.toRadians(-135))
//                .back(50)
//                .strafeRight(24)
//                .build();
//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(17)
//                .back(2)
//                .turn(Math.toRadians(45))
//                .back(10)
//                .build();////merge pt stanga pixel
//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(29.2)
//                .forward(26)
//                .strafeLeft(20)
//                .turn(Math.toRadians(-90))
//                .build();//blue side backboard middle scenario

        ////-------------Backdrop-------------

        TrajectorySequence Blue_BackDrop_Right = drive.trajectorySequenceBuilder(new Pose2d())//testata si merge
                //.addTemporalMarker(()->robot.gripper.closeBarier())
                .back(27)
                .turn(Math.toRadians(-90), 4,4)
                .lineToConstantHeading(new Vector2d(-25 , 6.8))
                .forward(10)
                .build();
        TrajectorySequence Blue_BackDrop_Right2  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right.end())
                //.addTemporalMarker(() -> ridicare())
                .lineToConstantHeading(new Vector2d(-10,-20))
                .turn(Math.toRadians(180))
                .build();
        TrajectorySequence Blue_BackDrop_Right3  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right2.end())
                //.addTemporalMarker(() -> robot.slider.raiseSlider(1600, 1))//era 1200
                .lineToConstantHeading(new Vector2d(-40,-42.5))
                .build();
        TrajectorySequence Blue_BackDrop_Right4  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right3.end())
                //.addTemporalMarker(() -> robot.arm.gripperReleasePos())
                .waitSeconds(2)
                .build();
        TrajectorySequence Blue_BackDrop_Right5  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right4.end())
                //.addTemporalMarker(()->robot.gripper.openBarier())
                .waitSeconds(1)
                .build();
        TrajectorySequence Blue_BackDrop_Right6  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right5.end())
                //.addTemporalMarker(()->robot.arm.gripperInitialPos())
                .waitSeconds(1)
                .build();
        TrajectorySequence Blue_BackDrop_Right7  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right6.end())
                //.addTemporalMarker(()->robot.slider.raiseSlider(0,1))
                .forward(7)//7
                .lineToConstantHeading(new Vector2d(-10,-37))
                .build();
        TrajectorySequence Blue_BackDrop_Right8  = drive.trajectorySequenceBuilder(Blue_BackDrop_Right7.end())
                //.addTemporalMarker(()->robot.arm.raiseArm(0,1))
                .waitSeconds(1)
                .build();
        TrajectorySequence Blue_BackDrop_Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-30,0))
                .lineToConstantHeading(new Vector2d(-20,0))
                .addTemporalMarker(()->{ robot.arm.raiseArm(900,1);})
                .lineToLinearHeading(new Pose2d(-15,-36, Math.toRadians(90)))//adauga outtake
                .addTemporalMarker(()->{ robot.slider.raiseSlider(1600,1);})
                .lineToConstantHeading(new Vector2d(-20,-37))
                .lineToConstantHeading(new Vector2d(-20,-41))

                .build();
        TrajectorySequence Blue_BackDrop_Left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-24,-9))//pixel on spike
                .lineToConstantHeading(new Vector2d(-2,-9))
                .lineToLinearHeading(new Pose2d(-15,-36, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-20,-37))
                .lineToConstantHeading(new Vector2d(-20,-41))



                //adauga outtake

                //parcare
//                .lineToConstantHeading(new Vector2d(-4,34))
//                .lineToConstantHeading(new Vector2d(-4,40))
                .build();

        ///// ---------Audience-----------

//        TrajectorySequence Red_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
//                .lineToConstantHeading(new Vector2d(-15,0))
//                .splineToLinearHeading(new Pose2d(-22,8.5,Math.toRadians(-75)),Math.toRadians(0))
//                .forward(12)
//                .lineToLinearHeading(new Pose2d(-57,4,Math.toRadians(-90)))
//                .back(69)
//                .strafeLeft(10)
//                .build();
        TrajectorySequence Red_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
               // .lineToConstantHeading(new Vector2d(-23,0))
               // .lineToConstantHeading(new Vector2d(-10,0))
                .back(27)
                .turn(Math.toRadians(-90), 4,4)
                .lineToConstantHeading(new Vector2d(-23 , 8))
                .lineTo(new Vector2d(-30, -10))
                .lineToConstantHeading(new Vector2d(-58 , 8))
                .lineToConstantHeading(new Vector2d(-58 , 78))
           //     .lineToConstantHeading(new Vector2d(-45 , 78))

//                .splineToLinearHeading(new Pose2d(-23,-10,Math.toRadians(-90)),Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(-23 , 18))
//                .lineToConstantHeading(new Vector2d(-23,0))
//                .strafeRight(18)
                .build();
        TrajectorySequence Red_Audience_Middle = drive.trajectorySequenceBuilder(new Pose2d())
//                .lineToConstantHeading(new Vector2d(-32,3))
//                .lineToConstantHeading(new Vector2d(-15,0))
//                .lineToConstantHeading(new Vector2d(-15,-14))
                .lineToLinearHeading(new Pose2d(-34,0,Math.toRadians(-90)))
                .forward(20)
                .lineToLinearHeading(new Pose2d(-50,-20,Math.toRadians(-85)))
                .lineToConstantHeading(new Vector2d(-58 , 50))
                //aduauga outtake
                .build();
        TrajectorySequence Red_Audience_Left =  drive.trajectorySequenceBuilder(new Pose2d())
//                .lineToConstantHeading(new Vector2d(-10 , -10))
//                .lineToLinearHeading(new Pose2d(-30 , -4 , Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d( -40 , 0 , Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-24,-9))//pixel on spike
                .lineToConstantHeading(new Vector2d(-2,-9))
                .lineToConstantHeading(new Vector2d(-5,-0))
                .lineToConstantHeading(new Vector2d(-52,-1))
                .lineToConstantHeading(new Vector2d(-52 ,1))
                .strafeLeft(65)
               // .lineToLinearHeading(new Pose2d(-35,78,Math.toRadians(-90)))
                .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.slider.raiseSlider(1000,1))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(1.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(1.5)
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.slider.raiseSlider(0,1))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .waitSeconds(2)
                    .build();




//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(24)
//                .strafeRight(15)
//                .forward(5)
//                .build();


        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(Blue_BackDrop_Middle);
//       // robot.gripper.closeBarier();
//        drive.followTrajectorySequence(Blue_BackDrop_Right2);
//      //  lastArmRaise = robot.arm.raiseArm(900,1);
//
//        drive.followTrajectorySequence(Blue_BackDrop_Right3);
//        //lastSliderRaise = robot.slider.raiseSlider(1600, 1);
//        drive.followTrajectorySequence(Blue_BackDrop_Right4);
//        //robot.arm.gripperReleasePos();
//        drive.followTrajectorySequence(Blue_BackDrop_Right5);
//        //robot.gripper.openBarier();
//        drive.followTrajectorySequence(Blue_BackDrop_Right6);
//        //robot.arm.gripperInitialPos();
//        drive.followTrajectorySequence(Blue_BackDrop_Right7);
//       // lastSliderRaise = robot.slider.raiseSlider(0,1);
//        drive.followTrajectorySequence(Blue_BackDrop_Right8);
//        //drive.followTrajectorySequence(
    }

//    public void ridicare() {
//        new Thread(() -> {
//            robot.arm.raiseArm(900,1);
//            sleep(4000);
//            robot.slider.raiseSlider(1600, 1);
//        }).start();
//    }



}