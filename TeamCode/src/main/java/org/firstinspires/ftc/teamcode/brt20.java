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

@Autonomous(name = "Brute 20 puncte Auto")
public class brt20 extends LinearOpMode {

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
            // test'
            waitForStart();
        }
        robot.slider.raiseSlider(200,0.5);
        sleep(1000);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (-1),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(350);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(200);
        robot.arm.raiseArm(600, 0.5);
        sleep(1000);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (-1),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(100);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(200);
        robot.slider.raiseSlider(4000, 1);
        sleep(2000);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (1),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(250);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(2000);
        robot.slider.raiseSlider(0, 0.5);
        sleep(1000);
        robot.arm.raiseArm(200, 0.5);
        sleep(1000);
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
