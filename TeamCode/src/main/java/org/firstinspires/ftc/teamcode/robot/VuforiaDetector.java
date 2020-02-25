package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

public class VuforiaDetector {
    private VuforiaLocalizer vuforia;

    private static final int STONE_WIDTH = 100, STONE_HEIGHT = 50, STONE_SPACING = 250;

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

    public VuforiaDetector(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = "AcRyVZf/////AAABmREbKz1DvE7yhFdOr9qQLSoq6MI/3Yoi9NEy+Z3poiBeamQswbGIX8ZqwRY5ElAoJe/4zqmdgZE73HPdbwsSDNk+9I17X4m8LGxRQaGOYsypI2HUvoFR+o141WvrzIYX2hhkANH7r+z5K0bY58wV6DUq3WCqN1fXWehixX956vv0wfXX2+YkVOo06U9llZwgmgE7gWKsgfcxmChr6PqXdiUtGsT4YztGG6Yr/c4Wlc6NDMIBgfmZWocJxl33oLpzO2DMkYWmgR3WOqsSBcjOEL2lvs5/D1UAVvuGe8uY6uMRjvZINIJznXnQbOJQrElTTT9G9mhjLR2ArCquvZbv/iCOh3k1DQMxsSkJXuyNAMle";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforia = ClassFactory.getInstance().createVuforia(params);

//        telemetry.addData("ID: ", cameraMonitorViewId);
//        telemetry.addData("hwMap: ", hardwareMap);
//        telemetry.addData("vuforia: ", vuforia);

//        telemetry.update();

        vuforia.enableConvertFrameToBitmap();
        vuforia.setFrameQueueCapacity(1);
    }

    public int runDetection(Telemetry telemetry) {
        Bitmap bmp = getBitmap();

        telemetry.addData("BMP W: ", bmp.getWidth());
        telemetry.addData("BMP H: ", bmp.getHeight());

//        int xCenter = (bmp.getWidth() - STONE_WIDTH) / 2;
        int yCenter = (bmp.getHeight() - STONE_HEIGHT) / 2;

        int leftMostX = (STONE_WIDTH / 2) + 150;

//        Bitmap stoneLeft = Bitmap.createBitmap(bmp, xCenter - STONE_SPACING, yCenter, STONE_WIDTH, STONE_HEIGHT);
//        Bitmap stoneCenter = Bitmap.createBitmap(bmp, xCenter, yCenter, STONE_WIDTH, STONE_HEIGHT);
//        Bitmap stoneRight = Bitmap.createBitmap(bmp, xCenter + STONE_SPACING, yCenter, STONE_WIDTH, STONE_HEIGHT);

        Bitmap stoneLeft = Bitmap.createBitmap(bmp, leftMostX, yCenter, STONE_WIDTH, STONE_HEIGHT);
        Bitmap stoneCenter = Bitmap.createBitmap(bmp, leftMostX + STONE_SPACING, yCenter, STONE_WIDTH, STONE_HEIGHT);
        Bitmap stoneRight = Bitmap.createBitmap(bmp, leftMostX + 2*STONE_SPACING, yCenter, STONE_WIDTH, STONE_HEIGHT);

        double ratioL = getColorVal(stoneLeft, ACTIVE_BLACK) / getColorVal(stoneLeft, ACTIVE_YELLOW);
        double ratioC = getColorVal(stoneCenter, ACTIVE_BLACK) / getColorVal(stoneCenter, ACTIVE_YELLOW);
        double ratioR = getColorVal(stoneRight, ACTIVE_BLACK) / getColorVal(stoneRight, ACTIVE_YELLOW);

        telemetry.addData("Left: ", ratioL);
        telemetry.addData("Center: ", ratioC);
        telemetry.addData("Right: ", ratioR);

        if (ratioL > ratioC && ratioL > ratioR) {
            telemetry.addLine("Skystone Left");
            telemetry.update();
            return 0;
        } else if (ratioC > ratioL && ratioC > ratioR) {
            telemetry.addLine("Skystone Center");
            telemetry.update();
            return 1;
        } else if (ratioR > ratioC && ratioR > ratioL) {
            telemetry.addLine("Skystone Right");
            telemetry.update();
            return 2;
        }
        return -1;
    }

    public Bitmap getBitmap() {
        Bitmap bitmap;
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = vuforia.getFrameQueue().take();
        } catch (Exception e) {
            throw new Warning("couldn't find vuforia frame");
        }

        bitmap = vuforia.convertFrameToBitmap(frame);
        return bitmap;
    }

/*
    public void saveBitmap(Bitmap bmp, String name) {
        String file_path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/images";
        telemetry.addData("file path: ", file_path);
        telemetry.update();

        File dir = new File(file_path);
        if(!dir.exists())
            dir.mkdirs();

        File[] files = dir.listFiles();

        int currentIdx = 0;
        for (int i = 0; i < files.length; i++) {
            String fileName = files[i].getName();
            if (fileName.indexOf(name) != -1) {
                int fileNum = Integer.parseInt(fileName.substring(name.length(), fileName.indexOf(".")));
                if(fileNum > currentIdx) currentIdx = fileNum;
            }
        }

        File file = new File(dir, name + (currentIdx+1) + ".png");

        FileOutputStream fOut = null;
        try {
            fOut = new FileOutputStream(file);
            bmp.compress(Bitmap.CompressFormat.PNG, 100, fOut);
            fOut.flush();
        } catch (FileNotFoundException ex) {
            throw new Warning("file not found");
        } catch (IOException ex) {
            throw new Warning("IOException");
        } finally {
            if (fOut != null) {
                try {
                    fOut.close();
                } catch (IOException ex) {
                    // ignore ... any significant errors should already have been
                    // reported via an IOException from the final flush.
                }
            }
        }
    }
*/

    private double getColorVal(Bitmap bitmap, ColorPreset colorPreset) {
        int color;
        int r, g, b;

        double normSum = 0;
        int pixels = bitmap.getWidth()*bitmap.getHeight();

        for (int i = 0; i < bitmap.getWidth(); i++) {
            for (int j = 0; j < bitmap.getHeight(); j++) {
                color = bitmap.getPixel(i, j);
                r = Color.red(color);
                g = Color.green(color);
                b = Color.blue(color);
                normSum += getColorNorm(r, g, b, colorPreset.r, colorPreset.g, colorPreset.b);
            }
        }

        double avgNorm = normSum/pixels;

        if (avgNorm != 0) {
            return 1/avgNorm;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    private double getColorNorm(int r, int g, int b, int targetR, int targetG, int targetB) {
        int rDiff = r - targetR;
        int gDiff = g - targetG;
        int bDiff = b - targetB;

        int rDiffSquared = (int) Math.pow(rDiff, 2);
        int gDiffSquared = (int) Math.pow(gDiff, 2);
        int bDiffSquared = (int) Math.pow(bDiff, 2);

        int sum = rDiffSquared + gDiffSquared + bDiffSquared;

        double norm = Math.sqrt(sum);

        return norm;
    }
}
