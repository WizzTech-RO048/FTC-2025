/**
 * Testing the implementation of the apriltag detection.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name = "Basket Auto")
public class AutoBasket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        ;

        ScheduledFuture<?> lastArmMove, lastSliderMove;

        while (!isStarted() && !isStopRequested()) {
            // test
            waitForStart();
        }

        int  arm_value = 175;
        double RAISE_POWER = 1.0;
        //robot.arm.raiseArm(arm_value, RAISE_POWER / 2);
        sleep(200);
        // TODO: @gorunescu uita te aici

        Pose2d startpose = new Pose2d(32, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence Blue_Basket = drive.trajectorySequenceBuilder(startpose)
                .addTemporalMarker(()->{robot.horizontalSlider.setStationaryPosition();})
                .splineToLinearHeading(new Pose2d(56, 55, Math.toRadians(-135)), Math.toRadians(20))
                .waitSeconds(0.8)
                .addTemporalMarker(()->{robot.gripper.score_object_release_position();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{robot.slider.raiseSlider(2700,1);})
                .waitSeconds(0.8)
                .back(8)
                .addTemporalMarker(()->{robot.gripper.outtake_release_position();})
                .waitSeconds(0.7)
                .forward(7)
                .addTemporalMarker(()->{robot.gripper.score_object_pickup_position();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{robot.slider.raiseSlider(0,1);})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(56, 56, Math.toRadians(-135)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(58, 48, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(56, 56, Math.toRadians(-135)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(58, 13, Math.toRadians(-90)))
                .strafeLeft(3)
                .lineToLinearHeading(new Pose2d(60, 50, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(37, 13, Math.toRadians(0)))
                .back(12)
                .build();

        drive.followTrajectorySequence(Blue_Basket);

        waitForStart();
        while (opModeIsActive()) {
            sleep(200);
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection tag) {
        Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

}
