/**
 * Testing the implementation of the apriltag detection.
 * */

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

@Autonomous(name="Human Auto")
public class AutoHuman extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );;

        ScheduledFuture<?> lastArmMove, lastSliderMove;

        while (!isStarted() && !isStopRequested()) {
            // test
            waitForStart();
        }

        TrajectorySequence Red_BackDrop_Left = drive.trajectorySequenceBuilder(new Pose2d())
//                .lineToConstantHeading(new Vector2d(-5,0))
//                .lineToConstantHeading(new Vector2d(-20,0))
//                .splineToLinearHeading(new Pose2d(-29,19,Math.toRadians(-90)),Math.toRadians(0))
//                .forward(5)
//                .back(12)
                .turn(Math.toRadians(-90))
//                .lineToConstantHeading(new Vector2d(-2,34))//parcare
//                .lineToConstantHeading(new Vector2d(0,40))
                .build();

        drive.followTrajectorySequence(Red_BackDrop_Left);


        while(opModeIsActive()) { sleep(200); }
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
