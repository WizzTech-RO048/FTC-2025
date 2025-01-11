

/**
 * Testing the implementation of the apriltag detection.
 * */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ComputerVision.Pipelines.TeamPropDetectionPipelineRed;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name="Red Backdrop Auto")
public class AutoRedBackdrop extends LinearOpMode {
    OpenCvCamera camera;
    TeamPropDetectionPipelineRed teamPropDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // ---- calibration for lens intrinsics, units are pixels ----
    // TODO: we need to do our own calibration for the camera, this one works for C920, 800x440
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;
    //ratio for optimized movement
    public static double ratioStrafe = (60.0/24.0)*0.94;
    public static double  ratioStraight = (60/48)*1.6;

    int detected_location;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );;

        ScheduledFuture<?> lastArmMove, lastSliderMove;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        teamPropDetectionPipeline = new TeamPropDetectionPipelineRed();

        camera.setPipeline(teamPropDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            detected_location = teamPropDetectionPipeline.getLocation();
            telemetry.addData("TeamProp Location", detected_location);

            telemetry.update();
            sleep(20);
        }

        if (detected_location == 1) {
            // scenariul left
            if(isStopRequested()) return;

            TrajectorySequence Red_BackDrop_Left = drive.trajectorySequenceBuilder(new Pose2d())
//                .lineToConstantHeading(new Vector2d(-20,4))
//                .splineToLinearHeading(new Pose2d(-22,-6, Math.toRadians(60)),Math.toRadians(0))
//                .forward(17)
//                .splineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(-90)), Math.toRadians(0))
                    .lineToConstantHeading(new Vector2d(-29,0))
                    .lineToConstantHeading(new Vector2d(-20,0))
                    .splineToLinearHeading(new Pose2d(-29,19,Math.toRadians(-90)),Math.toRadians(0))
                    .forward(18.5)
                    .back(12)
                    .turn(Math.toRadians(90))
                    //adauga outtake
                    //parcare
                    .lineToConstantHeading(new Vector2d(-2,34))//parcare
                    .lineToConstantHeading(new Vector2d(0,40))
                    .build();

            drive.followTrajectorySequence(Red_BackDrop_Left);

        } else if (detected_location == 2) {
            // scenariul mid
            TrajectorySequence Red_BackDrop_Middle = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(-32,0))
                    .lineToConstantHeading(new Vector2d(-20,0))
                    .splineToLinearHeading(new Pose2d(-4, 34, Math.toRadians(90)), Math.toRadians(0))//adauga outtake
                    .lineToConstantHeading(new Vector2d(-2,34))//parcare
                    .lineToConstantHeading(new Vector2d(0,40))
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(Red_BackDrop_Middle);

        } else if (detected_location == 3) {
            // scenariul right
            TrajectorySequence Red_BackDrop_Right = drive.trajectorySequenceBuilder(new Pose2d())//testata si merge
                    .lineToConstantHeading(new Vector2d(-22,17))//pixel on spike
                    .lineToConstantHeading(new Vector2d(-5,15.5))
                    .splineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(90)), Math.toRadians(0))//adauga outtake
                    .lineToConstantHeading(new Vector2d(0,34))//parcare
                    .lineToConstantHeading(new Vector2d(10,40))
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(Red_BackDrop_Right);

        }

        while(opModeIsActive()) { sleep(20); }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection tag) {
        Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

    public static double OptimizedStrafe(double x)
    {

        return  x*ratioStrafe;
    }
    public static double OptimizedStraight(double x){

        return x*ratioStraight;
    }
}
