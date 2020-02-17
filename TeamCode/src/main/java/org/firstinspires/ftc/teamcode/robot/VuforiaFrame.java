package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaFrame {

    private VuforiaLocalizer vuforiaLocalizer;

//    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

//    private static final float mmPerInch        = 25.4f;
//    private static final float stoneZ = 2.00f * mmPerInch;

    public VuforiaFrame(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId"," id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = "AcRyVZf/////AAABmREbKz1DvE7yhFdOr9qQLSoq6MI/3Yoi9NEy+Z3poiBeamQswbGIX8ZqwRY5ElAoJe/4zqmdgZE73HPdbwsSDNk+9I17X4m8LGxRQaGOYsypI2HUvoFR+o141WvrzIYX2hhkANH7r+z5K0bY58wV6DUq3WCqN1fXWehixX956vv0wfXX2+YkVOo06U9llZwgmgE7gWKsgfcxmChr6PqXdiUtGsT4YztGG6Yr/c4Wlc6NDMIBgfmZWocJxl33oLpzO2DMkYWmgR3WOqsSBcjOEL2lvs5/D1UAVvuGe8uY6uMRjvZINIJznXnQbOJQrElTTT9G9mhjLR2ArCquvZbv/iCOh3k1DQMxsSkJXuyNAMle";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(params);

        vuforiaLocalizer.enableConvertFrameToBitmap();
        vuforiaLocalizer.setFrameQueueCapacity(1);

/*
        VuforiaTrackables targetsSkyStone = this.vuforiaLocalizer.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, params.cameraDirection);
        }

        targetsSkyStone.activate();
*/
    }

    /**
     * Grabs a bitmap from the Vuforia engine for image processing
     * @return camera output as a bitmap
     */
    public Bitmap getBitmap(){
        Bitmap bitmap;
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = vuforiaLocalizer.getFrameQueue().take();
        } catch (Exception e) {
            throw new Warning("couldn't find vuforia frame");
        }

        bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
        return bitmap;
    }

/*
    public VuforiaLocalizer getVuforiaLocalizer() {
        return vuforiaLocalizer;
    }

    public VuforiaLocalizer.Parameters getParams() {
        return params;
    }
*/
}