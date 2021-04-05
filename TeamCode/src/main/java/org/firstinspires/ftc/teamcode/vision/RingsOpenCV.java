package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class RingsOpenCV {
    private OpenCvWebcam webcam;
    private SkystoneDeterminationPipeline pipeline;
    private WebcamName webcamName = null;

    private static final int STREAM_WIDTH = 640;
    private static final int STREAM_HEIGHT = 360;

    public RingsOpenCV(boolean isPowerShot, HardwareMap hwMap, Telemetry telemetry) {

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline(isPowerShot);
        webcam.setPipeline(pipeline);

//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Webcam Init Successful");
        telemetry.update();
    }

    public void analyzeRings(Telemetry telemetry) {
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
    }

    public int getAnalysis() {
        return pipeline.getAnalysis();
    }

    public int getNumberRings() {
        if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
            return 0;
        } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
            return 1;
        } else {
            return 4;
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        static int REGION_WIDTH = 100;
        static int REGION_HEIGHT = 80;
        static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point((STREAM_WIDTH - REGION_WIDTH) / 2 - 50, (STREAM_HEIGHT - REGION_HEIGHT) / 2 - 70);

        public SkystoneDeterminationPipeline(boolean isPowerShot)
        {
            if (isPowerShot) {
                REGION_WIDTH = 100;
                REGION_HEIGHT = 80;
                REGION1_TOPLEFT_ANCHOR_POINT = new Point(STREAM_WIDTH - REGION_WIDTH, (STREAM_HEIGHT - REGION_HEIGHT) / 2 - 70);
            }
        }

        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        final int FOUR_RING_THRESHOLD = 145;
        final int ONE_RING_THRESHOLD = 133;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

            position = RingPosition.NONE; // Record our analysis
            if (avg1 >= FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 >= ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}