/**
 * Testing the implementation of the apriltag detection.
 * */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.openftc.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name="Basket Auto Drop")
public class Auto_Basket_Drop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );;

        ScheduledFuture<?> lastArmMove, lastSliderMove;

        //robot.arm.raiseArm(100, 1);

        while (!isStarted() && !isStopRequested()) {
            // test'
            waitForStart();
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        (-0.5),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(500);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        //robot.arm.raiseArm(750,1);
        sleep(2000);
        robot.slider.raiseSlider(5700,1);
        sleep(3000);
        robot.gripper.intake_grab_position();
        sleep(2000);
        //robot.gripper.no_position();
        sleep(2000);
        //robot.arm.raiseArm(0,0.5);
        robot.slider.raiseSlider(0,1);
        sleep(5000);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0.5),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        sleep(150);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );
        //robot.arm.raiseArm(200, 0.5);
        sleep(2000);
        //robot.arm.raiseArm(0,1);
        drive.setWeightedDrivePower(
                new Pose2d(
                        (0),
                        (0),
                        (0)// gen astea negative / pozitive sau schimbate intre ele
                )
        );

        // safrsit test

        waitForStart();
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
