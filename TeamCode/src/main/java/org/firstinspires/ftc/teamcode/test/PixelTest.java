package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "PixelTest", group = "Test")
public class PixelTest extends LinearOpMode {

    private static final ColorPreset ACTIVE_YELLOW = ColorPreset.PURE_YELLOW;
    private static final ColorPreset ACTIVE_BLACK = ColorPreset.PURE_BLACK;

    private enum ColorPreset {
        PURE_YELLOW(255, 255, 0),
        PURE_BLACK(0, 0, 0);

        int r, g, b;

        ColorPreset(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    private static final String VUFORIA_KEY =
            "AcRyVZf/////AAABmREbKz1DvE7yhFdOr9qQLSoq6MI/3Yoi9NEy+Z3poiBeamQswbGIX8ZqwRY5ElAoJe/4zqmdgZE73HPdbwsSDNk+9I17X4m8LGxRQaGOYsypI2HUvoFR+o141WvrzIYX2hhkANH7r+z5K0bY58wV6DUq3WCqN1fXWehixX956vv0wfXX2+YkVOo06U9llZwgmgE7gWKsgfcxmChr6PqXdiUtGsT4YztGG6Yr/c4Wlc6NDMIBgfmZWocJxl33oLpzO2DMkYWmgR3WOqsSBcjOEL2lvs5/D1UAVvuGe8uY6uMRjvZINIJznXnQbOJQrElTTT9G9mhjLR2ArCquvZbv/iCOh3k1DQMxsSkJXuyNAMle";

    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    @Override public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = webcamName;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            vuforia.enableConvertFrameToBitmap();
            vuforia.setFrameQueueCapacity(1);

            telemetry.addData("ID: ", cameraMonitorViewId);
            telemetry.addData("hwMap: ", hardwareMap);
            telemetry.addData("vuforia: ", vuforia);

            telemetry.update();
        }
    }
}
