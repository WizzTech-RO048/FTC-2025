# WizzTech - FIRST Tech Challenge - INTO THE DEEP SEASON - 2024 / 2025

(This is the code of team WizzTech | RO_048 | 19094)@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Basket Auto")
public class AutoBasket extends LinearOpMode {
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int detected_location =1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        ScheduledFuture<?> lastArmMove, lastSliderMove;
        telemetry.setMsTransmissionInterval(50);      }// else if (detected_location == 2) {
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
    }@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Basket Auto")
    public class AutoBasket extends LinearOpMode {
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
    
        double tagsize = 0.166;
    
        int detected_location =1;
    
        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Robot robot = new Robot(
                    hardwareMap,
                    telemetry,
                    Executors.newScheduledThreadPool(1)
            );
            ScheduledFuture<?> lastArmMove, lastSliderMove;
            telemetry.setMsTransmissionInterval(50);      }// else if (detected_location == 2) {
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