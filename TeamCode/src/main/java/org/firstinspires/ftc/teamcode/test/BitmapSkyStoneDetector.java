package org.firstinspires.ftc.teamcode.test;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.VuforiaFrame;

@Autonomous(name="BmpSkystoneDetector", group="Test")
public class BitmapSkyStoneDetector extends LinearOpMode {
    VuforiaFrame vuforia;

    private static final int STONE_WIDTH = 100, STONE_HEIGHT = 25;

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

    @Override
    public void runOpMode() throws InterruptedException {
        vuforia = new VuforiaFrame(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Bitmap rgbBitmap = vuforia.getBitmap();
            int imgWidth = rgbBitmap.getWidth();
            int imgHeight = rgbBitmap.getHeight();

            Bitmap stoneLeft = Bitmap.createBitmap(rgbBitmap, (imgWidth - STONE_WIDTH) / 2 - (STONE_WIDTH + 50), (imgHeight - STONE_HEIGHT) / 2, STONE_WIDTH, STONE_HEIGHT);
            Bitmap stoneCenter = Bitmap.createBitmap(rgbBitmap, (imgWidth - STONE_WIDTH) / 2, (imgHeight - STONE_HEIGHT) / 2, STONE_WIDTH, STONE_HEIGHT);
            Bitmap stoneRight = Bitmap.createBitmap(rgbBitmap, (imgWidth - STONE_WIDTH) / 2 + (STONE_WIDTH + 50), (imgHeight - STONE_HEIGHT) / 2, STONE_WIDTH, STONE_HEIGHT);

            double ratioL = getColorVal(stoneLeft, ACTIVE_BLACK) / getColorVal(stoneLeft, ACTIVE_YELLOW);
            double ratioC = getColorVal(stoneCenter, ACTIVE_BLACK) / getColorVal(stoneCenter, ACTIVE_YELLOW);
            double ratioR = getColorVal(stoneRight, ACTIVE_BLACK) / getColorVal(stoneRight, ACTIVE_YELLOW);

            telemetry.addData("Left: ", ratioL);
            telemetry.addData("Center: ", ratioC);
            telemetry.addData("Right: ", ratioR);

            if (ratioL > ratioC && ratioL > ratioR) {
                telemetry.addLine("Skystone Left");
            } else if (ratioC > ratioL && ratioC > ratioR) {
                telemetry.addLine("Skystone Center");
            } else {
                telemetry.addLine("Skystone Right");
            }
            telemetry.update();

            sleep(1000);
        }
    }

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
