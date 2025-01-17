package org.firstinspires.ftc.teamcode; /**
 * Testing the implementation of the apriltag detection.
 * */

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name="Basket Auto")
public class AutoBasket extends LinearOpMode {
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int detected_location;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        ScheduledFuture<?> lastArmMove, lastSliderMove;
        telemetry.setMsTransmissionInterval(50);


//        if (detected_location == 1) {///-------------unghi pe sub schela
//            TrajectorySequence Blue_Audience_Left = drive.trajectorySequenceBuilder(new Pose2d())
//                    .lineToConstantHeading(new Vector2d(-29,0))
//                    .lineToConstantHeading(new Vector2d(-20,0))
//                    .splineToLinearHeading(new Pose2d(-29,19,Math.toRadians(-90)),Math.toRadians(0))
//                    .forward(21)
//                    .back(12)
//                    .build();
//
//            if(isStopRequested()) return;
//            drive.followTrajectorySequence(Blue_Audience_Left);
//
//        } else if (detected_location == 2) {
//            // scenariul mid
//            TrajectorySequence Blue_Audience_Middle = drive.trajectorySequenceBuilder(new Pose2d())
//                    .back(32)
//                    .forward(10)
//                    .strafeLeft(15)
//                    .build();
//
//            if(isStopRequested()) return;
//            drive.followTrajectorySequence(Blue_Audience_Middle);
//
//        } else if (detected_location == 3) {
//            // scenariul right
//            TrajectorySequence Blue_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
//                    .lineToConstantHeading(new Vector2d(-24,18))//pixel on spike
//                    .lineToConstantHeading(new Vector2d(-2,20))
//                    .lineToConstantHeading(new Vector2d(-5,1))
//                    .turn(-Math.toRadians(10))
//                    .build();
//            if(isStopRequested()) return;
//            drive.followTrajectorySequence(Blue_Audience_Right);
//        }

        while(opModeIsActive()) { sleep(20); }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection tag) {
        Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

//        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

}
