package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

public class TelemetryFormats{

    Telemetry telemetry;

    public TelemetryFormats(Telemetry t) {
        telemetry = t;
    }

    public void aprilTagToTelemetry(AprilTagDetection tag) {
        telemetry.addData("tag id", tag.id);
    }


}
